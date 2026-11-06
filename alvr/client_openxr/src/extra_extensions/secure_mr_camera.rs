// Headset camera access through PICO SecureMR (XR_PICO_secure_mixed_reality) and tensor readback
// (XR_PICO_readback_tensor). One pipeline run produces the rectified left camera image, its
// timestamp, its intrinsics and the camera pose in the XR LOCAL space, all for the same exposure.
// Readback requires the Android CAMERA permission.
//
// The openxr crate has no bindings for these PICO extensions: the types below mirror PICO's
// openxr.h (SecureMR-Samples, external/openxr/include/openxr/openxr.h).

use alvr_packets::CameraEye;
use openxr::{self as xr, sys};
use std::{
    ffi::{CStr, c_void},
    ptr, thread,
    time::{Duration, Instant},
};

pub const SECURE_MR_EXTENSION_NAME: &str = "XR_PICO_secure_mixed_reality";
pub const READBACK_TENSOR_EXTENSION_NAME: &str = "XR_PICO_readback_tensor";

const TYPE_FRAMEWORK_CREATE_INFO: i32 = 1_010_007_000;
const TYPE_PIPELINE_CREATE_INFO: i32 = 1_010_007_001;
const TYPE_OPERATOR_CREATE_INFO: i32 = 1_010_007_003;
const TYPE_TENSOR_CREATE_INFO_SHAPE: i32 = 1_010_007_013;
const TYPE_PIPELINE_EXECUTE_PARAMETER: i32 = 1_010_007_016;
const TYPE_PIPELINE_IO_PAIR: i32 = 1_010_007_017;
const TYPE_READBACK_TENSOR_BUFFER: i32 = 1_010_027_000;
const TYPE_CREATE_BUFFER_FROM_GLOBAL_TENSOR_COMPLETION: i32 = 1_010_027_001;

const OPERATOR_CAMERA_SPACE_TO_WORLD: i32 = 22;
const OPERATOR_RECTIFIED_VST_ACCESS: i32 = 23;

const DATA_TYPE_UINT8: i32 = 1;
const DATA_TYPE_INT32: i32 = 5;
const DATA_TYPE_FLOAT32: i32 = 6;
const TENSOR_TYPE_TIMESTAMP: i32 = 5;
const TENSOR_TYPE_MAT: i32 = 6;

const READBACK_POLL_INTERVAL: Duration = Duration::from_millis(1);
const READBACK_TIMEOUT: Duration = Duration::from_secs(2);

type Handle = u64;

#[repr(C)]
struct FrameworkCreateInfo {
    ty: sys::StructureType,
    next: *const c_void,
    width: i32,
    height: i32,
}

#[repr(C)]
struct PipelineCreateInfo {
    ty: sys::StructureType,
    next: *const c_void,
}

#[repr(C)]
struct OperatorCreateInfo {
    ty: sys::StructureType,
    next: *const c_void,
    operator_info: *const c_void,
    operator_type: i32,
}

#[repr(C)]
struct TensorFormat {
    data_type: i32,
    channel: i8,
    tensor_type: i32,
}

#[repr(C)]
struct TensorCreateInfoShape {
    ty: sys::StructureType,
    next: *const c_void,
    place_holder: sys::Bool32,
    dimensions_count: u32,
    dimensions: *const i32,
    format: *const TensorFormat,
}

#[repr(C)]
struct PipelineIoPair {
    ty: sys::StructureType,
    next: *const c_void,
    local_place_holder_tensor: Handle,
    global_tensor: Handle,
}

#[repr(C)]
struct PipelineExecuteParameter {
    ty: sys::StructureType,
    next: *const c_void,
    pipeline_run_to_be_waited: Handle,
    condition_tensor: Handle,
    pair_count: u32,
    pipeline_io_pair: *const PipelineIoPair,
}

#[repr(C)]
struct ReadbackTensorBuffer {
    ty: sys::StructureType,
    next: *const c_void,
    buffer_capacity_input: u32,
    buffer_count_output: u32,
    buffer: *mut c_void,
}

#[repr(C)]
struct CreateBufferCompletion {
    ty: sys::StructureType,
    next: *mut c_void,
    future_result: sys::Result,
    tensor_buffer: *mut ReadbackTensorBuffer,
}

type CreateFrameworkFn =
    unsafe extern "system" fn(sys::Session, *const FrameworkCreateInfo, *mut Handle) -> sys::Result;
type DestroyHandleFn = unsafe extern "system" fn(Handle) -> sys::Result;
type CreatePipelineFn =
    unsafe extern "system" fn(Handle, *const PipelineCreateInfo, *mut Handle) -> sys::Result;
type CreateOperatorFn =
    unsafe extern "system" fn(Handle, *const OperatorCreateInfo, *mut Handle) -> sys::Result;
type CreateTensorFn =
    unsafe extern "system" fn(Handle, *const TensorCreateInfoShape, *mut Handle) -> sys::Result;
type SetOperatorTensorByNameFn =
    unsafe extern "system" fn(Handle, Handle, Handle, *const std::ffi::c_char) -> sys::Result;
type ExecutePipelineFn =
    unsafe extern "system" fn(Handle, *const PipelineExecuteParameter, *mut Handle) -> sys::Result;
type CreateBufferAsyncFn = unsafe extern "system" fn(Handle, *mut sys::FutureEXT) -> sys::Result;
type CreateBufferCompleteFn =
    unsafe extern "system" fn(Handle, sys::FutureEXT, *mut CreateBufferCompletion) -> sys::Result;

struct Functions {
    create_framework: CreateFrameworkFn,
    destroy_framework: DestroyHandleFn,
    create_pipeline: CreatePipelineFn,
    destroy_pipeline: DestroyHandleFn,
    create_operator: CreateOperatorFn,
    create_tensor: CreateTensorFn,
    destroy_tensor: DestroyHandleFn,
    create_pipeline_tensor: CreateTensorFn,
    set_operand_by_name: SetOperatorTensorByNameFn,
    set_result_by_name: SetOperatorTensorByNameFn,
    execute_pipeline: ExecutePipelineFn,
    create_buffer_async: CreateBufferAsyncFn,
    create_buffer_complete: CreateBufferCompleteFn,
}

// Tensor shape and format: dimensions, channels, data type, tensor type
type TensorDesc<'a> = (&'a [i32], i8, i32, i32);

const TIMESTAMP_DESC: TensorDesc<'static> = (&[1], 4, DATA_TYPE_INT32, TENSOR_TYPE_TIMESTAMP);
const CAMERA_MATRIX_DESC: TensorDesc<'static> = (&[3, 3], 1, DATA_TYPE_FLOAT32, TENSOR_TYPE_MAT);
const TRANSFORM_DESC: TensorDesc<'static> = (&[4, 4], 1, DATA_TYPE_FLOAT32, TENSOR_TYPE_MAT);

// One image with the metadata of the same exposure
pub struct CameraCapture {
    pub size: u32,
    /// Row-major RGB, 3 bytes per pixel, size * size pixels
    pub rgb: Vec<u8>,
    /// Capture time on the headset CLOCK_MONOTONIC
    pub timestamp: Duration,
    /// Row-major 3x3 intrinsics of the image
    pub camera_matrix: [f32; 9],
    /// Row-major 4x4 camera -> XR LOCAL space, OpenXR camera axes (+Y up, -Z forward)
    pub local_from_camera: [f32; 16],
}

pub struct SecureMrCamera {
    session: xr::Session<xr::AnyGraphics>,
    functions: Functions,
    size: u32,
    eye: CameraEye,
    framework: Handle,
    pipeline: Handle,
    // image, timestamp, camera matrix, transform
    global_tensors: [Handle; 4],
    placeholders: [Handle; 4],
}

// The handles are only used from the thread that owns the camera
unsafe impl Send for SecureMrCamera {}

fn load<G, T>(session: &xr::Session<G>, name: &str) -> Result<T, String> {
    super::get_instance_proc(session, name).map_err(|e| format!("{name}: {e}"))
}

impl SecureMrCamera {
    pub fn new<G>(session: &xr::Session<G>, size: u32, eye: CameraEye) -> Result<Self, String> {
        let functions = Functions {
            create_framework: load(session, "xrCreateSecureMrFrameworkPICO")?,
            destroy_framework: load(session, "xrDestroySecureMrFrameworkPICO")?,
            create_pipeline: load(session, "xrCreateSecureMrPipelinePICO")?,
            destroy_pipeline: load(session, "xrDestroySecureMrPipelinePICO")?,
            create_operator: load(session, "xrCreateSecureMrOperatorPICO")?,
            create_tensor: load(session, "xrCreateSecureMrTensorPICO")?,
            destroy_tensor: load(session, "xrDestroySecureMrTensorPICO")?,
            create_pipeline_tensor: load(session, "xrCreateSecureMrPipelineTensorPICO")?,
            set_operand_by_name: load(session, "xrSetSecureMrOperatorOperandByNamePICO")?,
            set_result_by_name: load(session, "xrSetSecureMrOperatorResultByNamePICO")?,
            execute_pipeline: load(session, "xrExecuteSecureMrPipelinePICO")?,
            create_buffer_async: load(session, "xrCreateBufferFromGlobalTensorAsyncPICO")?,
            create_buffer_complete: load(session, "xrCreateBufferFromGlobalTensorCompletePICO")?,
        };

        let mut this = Self {
            session: session.clone().into_any_graphics(),
            functions,
            size,
            eye,
            framework: 0,
            pipeline: 0,
            global_tensors: [0; 4],
            placeholders: [0; 4],
        };
        // On error, Drop releases whatever was created so far
        this.create_objects()
            .map_err(|e| format!("creating the SecureMR pipeline: {e}"))?;

        Ok(this)
    }

    fn create_objects(&mut self) -> xr::Result<()> {
        let f = &self.functions;
        let side = self.size as i32;

        let framework_info = FrameworkCreateInfo {
            ty: sys::StructureType::from_raw(TYPE_FRAMEWORK_CREATE_INFO),
            next: ptr::null(),
            width: side,
            height: side,
        };
        super::xr_res(unsafe {
            (f.create_framework)(
                self.session.as_raw(),
                &raw const framework_info,
                &raw mut self.framework,
            )
        })?;

        let pipeline_info = PipelineCreateInfo {
            ty: sys::StructureType::from_raw(TYPE_PIPELINE_CREATE_INFO),
            next: ptr::null(),
        };
        super::xr_res(unsafe {
            (f.create_pipeline)(
                self.framework,
                &raw const pipeline_info,
                &raw mut self.pipeline,
            )
        })?;

        let image_dimensions = [side, side];
        let descs: [TensorDesc; 4] = [
            (&image_dimensions, 3, DATA_TYPE_UINT8, TENSOR_TYPE_MAT),
            TIMESTAMP_DESC,
            CAMERA_MATRIX_DESC,
            TRANSFORM_DESC,
        ];
        for (index, (dimensions, channels, data_type, tensor_type)) in descs.into_iter().enumerate()
        {
            let format = TensorFormat {
                data_type,
                channel: channels,
                tensor_type,
            };
            let mut info = TensorCreateInfoShape {
                ty: sys::StructureType::from_raw(TYPE_TENSOR_CREATE_INFO_SHAPE),
                next: ptr::null(),
                place_holder: sys::FALSE,
                dimensions_count: dimensions.len() as u32,
                dimensions: dimensions.as_ptr(),
                format: &raw const format,
            };
            // # Safety: index < 4 (four descriptors)
            super::xr_res(unsafe {
                (f.create_tensor)(
                    self.framework,
                    &raw const info,
                    &raw mut self.global_tensors[index],
                )
            })?;

            info.place_holder = sys::TRUE;
            super::xr_res(unsafe {
                (f.create_pipeline_tensor)(
                    self.pipeline,
                    &raw const info,
                    &raw mut self.placeholders[index],
                )
            })?;
        }
        let [image, timestamp, camera_matrix, transform] = self.placeholders;

        let vst_access = self.create_operator(OPERATOR_RECTIFIED_VST_ACCESS)?;
        let (image_name, transform_name) = match self.eye {
            CameraEye::Left => (c"left image", c"left"),
            CameraEye::Right => (c"right image", c"right"),
        };
        self.set_result(vst_access, image, image_name)?;
        self.set_result(vst_access, timestamp, c"timestamp")?;
        self.set_result(vst_access, camera_matrix, c"camera matrix")?;

        let camera_to_world = self.create_operator(OPERATOR_CAMERA_SPACE_TO_WORLD)?;
        super::xr_res(unsafe {
            (self.functions.set_operand_by_name)(
                self.pipeline,
                camera_to_world,
                timestamp,
                c"timestamp".as_ptr(),
            )
        })?;
        self.set_result(camera_to_world, transform, transform_name)?;

        Ok(())
    }

    fn create_operator(&self, operator_type: i32) -> xr::Result<Handle> {
        let info = OperatorCreateInfo {
            ty: sys::StructureType::from_raw(TYPE_OPERATOR_CREATE_INFO),
            next: ptr::null(),
            operator_info: ptr::null(),
            operator_type,
        };
        let mut operator = 0;
        super::xr_res(unsafe {
            (self.functions.create_operator)(self.pipeline, &raw const info, &raw mut operator)
        })?;

        Ok(operator)
    }

    fn set_result(&self, operator: Handle, tensor: Handle, name: &CStr) -> xr::Result<()> {
        super::xr_res(unsafe {
            (self.functions.set_result_by_name)(self.pipeline, operator, tensor, name.as_ptr())
        })
    }

    // Copies a global tensor into app memory. Blocks until the pipelines writing it are done.
    fn read_back(&self, tensor: Handle) -> xr::Result<Vec<u8>> {
        let f = &self.functions;

        let mut future = sys::FutureEXT::from_raw(0);
        super::xr_res(unsafe { (f.create_buffer_async)(tensor, &raw mut future) })?;

        let deadline = Instant::now() + READBACK_TIMEOUT;
        while !super::check_future(self.session.instance(), future)? {
            if Instant::now() > deadline {
                return Err(sys::Result::TIMEOUT_EXPIRED);
            }
            thread::sleep(READBACK_POLL_INTERVAL);
        }

        // Two-call idiom: query the size, then fill the buffer
        let mut buffer = ReadbackTensorBuffer {
            ty: sys::StructureType::from_raw(TYPE_READBACK_TENSOR_BUFFER),
            next: ptr::null(),
            buffer_capacity_input: 0,
            buffer_count_output: 0,
            buffer: ptr::null_mut(),
        };
        let mut completion = CreateBufferCompletion {
            ty: sys::StructureType::from_raw(TYPE_CREATE_BUFFER_FROM_GLOBAL_TENSOR_COMPLETION),
            next: ptr::null_mut(),
            future_result: sys::Result::SUCCESS,
            tensor_buffer: &raw mut buffer,
        };
        super::xr_res(unsafe { (f.create_buffer_complete)(tensor, future, &raw mut completion) })?;

        let mut data = vec![0_u8; buffer.buffer_count_output as usize];
        buffer.buffer_capacity_input = buffer.buffer_count_output;
        buffer.buffer = data.as_mut_ptr().cast();
        completion.tensor_buffer = &raw mut buffer;
        super::xr_res(unsafe { (f.create_buffer_complete)(tensor, future, &raw mut completion) })?;
        super::xr_res(completion.future_result)?;

        data.truncate(buffer.buffer_count_output as usize);

        Ok(data)
    }

    // Runs the pipeline once and reads back image and metadata of that run
    pub fn capture(&self) -> xr::Result<CameraCapture> {
        let pairs = self
            .placeholders
            .iter()
            .zip(self.global_tensors)
            .map(|(&placeholder, global)| PipelineIoPair {
                ty: sys::StructureType::from_raw(TYPE_PIPELINE_IO_PAIR),
                next: ptr::null(),
                local_place_holder_tensor: placeholder,
                global_tensor: global,
            })
            .collect::<Vec<_>>();
        let parameter = PipelineExecuteParameter {
            ty: sys::StructureType::from_raw(TYPE_PIPELINE_EXECUTE_PARAMETER),
            next: ptr::null(),
            pipeline_run_to_be_waited: 0,
            condition_tensor: 0,
            pair_count: pairs.len() as u32,
            pipeline_io_pair: pairs.as_ptr(),
        };
        let mut run = 0;
        super::xr_res(unsafe {
            (self.functions.execute_pipeline)(self.pipeline, &raw const parameter, &raw mut run)
        })?;

        let [image, timestamp, camera_matrix, transform] = self.global_tensors;
        let rgb = self.read_back(image)?;
        let timestamp = self.read_back(timestamp)?;
        let camera_matrix = self.read_back(camera_matrix)?;
        let transform = self.read_back(transform)?;

        let pixel_bytes = self.size as usize * self.size as usize * 3;
        if rgb.len() != pixel_bytes
            || timestamp.len() < 16
            || camera_matrix.len() < 36
            || transform.len() < 64
        {
            return Err(sys::Result::ERROR_VALIDATION_FAILURE);
        }

        let int32_at = |bytes: &[u8], index: usize| {
            // # Safety: callers stay within the sizes checked above
            i32::from_le_bytes(bytes[index * 4..index * 4 + 4].try_into().unwrap())
        };
        let float32_at = |bytes: &[u8], index: usize| f32::from_bits(int32_at(bytes, index) as u32);

        // {0, seconds, 0, nanoseconds}
        let timestamp = Duration::new(
            int32_at(&timestamp, 1).max(0) as u64,
            int32_at(&timestamp, 3).clamp(0, 999_999_999) as u32,
        );

        Ok(CameraCapture {
            size: self.size,
            rgb,
            timestamp,
            camera_matrix: std::array::from_fn(|i| float32_at(&camera_matrix, i)),
            local_from_camera: std::array::from_fn(|i| float32_at(&transform, i)),
        })
    }
}

impl Drop for SecureMrCamera {
    fn drop(&mut self) {
        let f = &self.functions;
        unsafe {
            if self.pipeline != 0 {
                (f.destroy_pipeline)(self.pipeline);
            }
            for tensor in self.global_tensors {
                if tensor != 0 {
                    (f.destroy_tensor)(tensor);
                }
            }
            if self.framework != 0 {
                (f.destroy_framework)(self.framework);
            }
        }
    }
}
