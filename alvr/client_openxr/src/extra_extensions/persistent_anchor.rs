// One persistent spatial anchor (normally at the overlay app's touchscreen), through the standard
// XR_EXT_spatial_entity + XR_EXT_spatial_anchor + XR_EXT_spatial_persistence(_operations)
// extensions (LOCAL_ANCHORS scope: same device, same app). Polled once per frame, never blocks:
// every runtime call that may take time is a future checked on the next polls.
//
// Life cycle: persistence context -> spatial context (anchor capability, persistence) -> if a UUID
// was persisted before, discover it -> locate it in STAGE every LOCATE_INTERVAL. Commands: create
// (or replace) the anchor at a STAGE pose, forget it.

use alvr_common::{Pose, info, warn};
use openxr::{self as xr, raw, sys, sys::Handle as _};
use std::{
    ptr,
    time::{Duration, Instant},
};

const LOCATE_INTERVAL: Duration = Duration::from_millis(500);
// A discovery that finds nothing is retried this often (the map may relocalise later)
const DISCOVERY_RETRY_INTERVAL: Duration = Duration::from_secs(5);
const PERMISSION_RETRY_INTERVAL: Duration = Duration::from_secs(10);

// VrAnchorState values of the shared-memory contract (ScreenVR VrTrackingExtras.h)
pub const ANCHOR_NONE: u32 = 0;
pub const ANCHOR_CREATING: u32 = 1;
pub const ANCHOR_LOCATED: u32 = 2;
pub const ANCHOR_LOST: u32 = 3;
pub const ANCHOR_UNSUPPORTED: u32 = 4;
pub const ANCHOR_ERROR: u32 = 5;

#[derive(Clone, Copy, Debug)]
pub enum AnchorCommand {
    Create(Pose),
    Forget,
}

#[derive(Clone, Debug, Default)]
pub struct AnchorStatus {
    pub state: u32,
    pub uuid: [u8; 16],
    // In STAGE: current when located, last good pose when lost
    pub pose: Option<Pose>,
    pub generation: u32,
}

impl PartialEq for AnchorStatus {
    fn eq(&self, other: &Self) -> bool {
        self.state == other.state
            && self.uuid == other.uuid
            && self.generation == other.generation
            && match (self.pose, other.pose) {
                (Some(a), Some(b)) => a.position == b.position && a.orientation == b.orientation,
                (None, None) => true,
                _ => false,
            }
    }
}

enum Phase {
    CreatingPersistence(sys::FutureEXT),
    CreatingContext(sys::FutureEXT),
    // Spatial data permission not granted (yet): the spatial context is retried at this time
    WaitingPermission(Instant),
    Ready,
    Failed,
}

enum Pending {
    None,
    Discover(sys::FutureEXT),
    // New anchor entity being persisted; the UUID to unpersist afterwards (replace)
    Persist(
        sys::FutureEXT,
        sys::SpatialEntityIdEXT,
        sys::SpatialEntityEXT,
        Option<[u8; 16]>,
    ),
    Unpersist(sys::FutureEXT),
}

pub struct PersistentAnchor {
    instance: xr::Instance,
    session_handle: sys::Session,
    entity_fns: raw::SpatialEntityEXT,
    anchor_fns: raw::SpatialAnchorEXT,
    persistence_fns: raw::SpatialPersistenceEXT,
    operations_fns: raw::SpatialPersistenceOperationsEXT,
    phase: Phase,
    persistence_context: sys::SpatialPersistenceContextEXT,
    context: sys::SpatialContextEXT,
    entity: Option<(sys::SpatialEntityIdEXT, sys::SpatialEntityEXT)>,
    pending: Pending,
    command: Option<AnchorCommand>,
    last_locate: Option<Instant>,
    last_discovery: Option<Instant>,
    last_locate_outcome: String,
    last_query_outcome: String,
    pub status: AnchorStatus,
}

fn uuid_hex(uuid: &[u8; 16]) -> String {
    uuid.iter().map(|b| format!("{b:02x}")).collect()
}

impl PersistentAnchor {
    /// None when the runtime lacks one of the extensions (status: unsupported)
    pub fn new(
        session: &xr::Session<xr::OpenGlEs>,
        stored_uuid: Option<[u8; 16]>,
        generation: u32,
    ) -> Option<Self> {
        let instance = session.instance().clone();
        let exts = instance.exts();
        let (Some(entity_fns), Some(anchor_fns), Some(persistence_fns), Some(operations_fns)) = (
            exts.ext_spatial_entity,
            exts.ext_spatial_anchor,
            exts.ext_spatial_persistence,
            exts.ext_spatial_persistence_operations,
        ) else {
            info!(
                "Anchor: unsupported (spatial entity {}, anchor {}, persistence {}, persistence \
                 operations {})",
                exts.ext_spatial_entity.is_some(),
                exts.ext_spatial_anchor.is_some(),
                exts.ext_spatial_persistence.is_some(),
                exts.ext_spatial_persistence_operations.is_some()
            );
            return None;
        };

        let create_info = sys::SpatialPersistenceContextCreateInfoEXT {
            ty: sys::SpatialPersistenceContextCreateInfoEXT::TYPE,
            next: ptr::null(),
            scope: sys::SpatialPersistenceScopeEXT::LOCAL_ANCHORS,
        };
        let mut future = sys::FutureEXT::from_raw(0);
        let result = unsafe {
            (persistence_fns.create_spatial_persistence_context_async)(
                session.as_raw(),
                &raw const create_info,
                &raw mut future,
            )
        };
        let phase = if result == sys::Result::SUCCESS {
            Phase::CreatingPersistence(future)
        } else {
            warn!("Anchor: xrCreateSpatialPersistenceContextAsyncEXT failed: {result:?}");
            Phase::Failed
        };
        info!(
            "Anchor: starting, stored anchor {}",
            stored_uuid.as_ref().map_or("none".into(), uuid_hex)
        );

        Some(Self {
            instance,
            session_handle: session.as_raw(),
            entity_fns,
            anchor_fns,
            persistence_fns,
            operations_fns,
            phase,
            persistence_context: sys::SpatialPersistenceContextEXT::NULL,
            context: sys::SpatialContextEXT::NULL,
            entity: None,
            pending: Pending::None,
            command: None,
            last_locate: None,
            last_discovery: None,
            last_locate_outcome: String::new(),
            last_query_outcome: String::new(),
            status: AnchorStatus {
                state: if result == sys::Result::SUCCESS {
                    if stored_uuid.is_some() {
                        ANCHOR_LOST
                    } else {
                        ANCHOR_NONE
                    }
                } else {
                    ANCHOR_ERROR
                },
                uuid: stored_uuid.unwrap_or([0; 16]),
                pose: None,
                generation,
            },
        })
    }

    /// Carried out as soon as the anchor machinery is ready; a newer command replaces a waiting one
    pub fn command(&mut self, command: AnchorCommand) {
        self.command = Some(command);
    }

    fn fail(&mut self, what: &str, result: sys::Result) {
        warn!("Anchor: {what} failed: {result:?}");
        self.status.state = ANCHOR_ERROR;
    }

    fn future_ready(&self, future: sys::FutureEXT) -> bool {
        super::check_future(&self.instance, future).unwrap_or(false)
    }

    /// Once per frame. Returns true when the status changed.
    pub fn poll(&mut self, stage: &xr::Space, time: xr::Time) -> bool {
        let before = self.status.clone();
        match self.phase {
            Phase::Failed => (),
            Phase::CreatingPersistence(future) => self.poll_persistence(future),
            Phase::CreatingContext(future) => self.poll_context(future),
            Phase::WaitingPermission(retry) => {
                if Instant::now() >= retry {
                    self.start_context();
                }
            }
            Phase::Ready => self.poll_ready(stage, time),
        }

        self.status != before
    }

    fn poll_persistence(&mut self, future: sys::FutureEXT) {
        if !self.future_ready(future) {
            return;
        }
        let mut completion =
            sys::CreateSpatialPersistenceContextCompletionEXT::out(ptr::null_mut());
        let result = unsafe {
            (self
                .persistence_fns
                .create_spatial_persistence_context_complete)(
                self.session_handle,
                future,
                completion.as_mut_ptr(),
            )
        };
        let completion = unsafe { completion.assume_init() };
        if result != sys::Result::SUCCESS || completion.future_result != sys::Result::SUCCESS {
            self.phase = Phase::Failed;
            warn!(
                "Anchor: persistence context failed: {result:?} / {:?} / {:?}",
                completion.future_result, completion.create_result
            );
            self.status.state = ANCHOR_UNSUPPORTED;
            return;
        }
        self.persistence_context = completion.persistence_context;
        self.start_context();
    }

    fn start_context(&mut self) {
        let components = [
            sys::SpatialComponentTypeEXT::ANCHOR,
            sys::SpatialComponentTypeEXT::PERSISTENCE,
        ];
        let anchor_config = sys::SpatialCapabilityConfigurationAnchorEXT {
            ty: sys::SpatialCapabilityConfigurationAnchorEXT::TYPE,
            next: ptr::null(),
            capability: sys::SpatialCapabilityEXT::ANCHOR,
            enabled_component_count: components.len() as u32,
            enabled_components: components.as_ptr(),
        };
        let config_ptr = ptr::from_ref(&anchor_config).cast();
        let persistence_config = sys::SpatialContextPersistenceConfigEXT {
            ty: sys::SpatialContextPersistenceConfigEXT::TYPE,
            next: ptr::null(),
            persistence_context_count: 1,
            persistence_contexts: &raw const self.persistence_context,
        };
        let create_info = sys::SpatialContextCreateInfoEXT {
            ty: sys::SpatialContextCreateInfoEXT::TYPE,
            next: ptr::from_ref(&persistence_config).cast(),
            capability_config_count: 1,
            capability_configs: &raw const config_ptr,
        };
        let mut future = sys::FutureEXT::from_raw(0);
        let result = unsafe {
            (self.entity_fns.create_spatial_context_async)(
                self.session_handle,
                &raw const create_info,
                &raw mut future,
            )
        };
        if result == sys::Result::SUCCESS {
            self.phase = Phase::CreatingContext(future);
        } else {
            self.phase = Phase::Failed;
            self.fail("xrCreateSpatialContextAsyncEXT", result);
        }
    }

    fn poll_context(&mut self, future: sys::FutureEXT) {
        if !self.future_ready(future) {
            return;
        }
        let mut completion = sys::CreateSpatialContextCompletionEXT::out(ptr::null_mut());
        let result = unsafe {
            (self.entity_fns.create_spatial_context_complete)(
                self.session_handle,
                future,
                completion.as_mut_ptr(),
            )
        };
        let completion = unsafe { completion.assume_init() };
        if result == sys::Result::SUCCESS
            && completion.future_result == sys::Result::ERROR_PERMISSION_INSUFFICIENT
        {
            // PICO: com.picovr.permission.SPATIAL_DATA (runtime permission, may be granted later)
            if self.status.state != ANCHOR_UNSUPPORTED {
                warn!(
                    "Anchor: spatial data permission not granted, retrying every {} s",
                    PERMISSION_RETRY_INTERVAL.as_secs()
                );
            }
            self.phase = Phase::WaitingPermission(Instant::now() + PERMISSION_RETRY_INTERVAL);
            self.status.state = ANCHOR_UNSUPPORTED;
            return;
        }
        if result != sys::Result::SUCCESS || completion.future_result != sys::Result::SUCCESS {
            self.phase = Phase::Failed;
            warn!(
                "Anchor: spatial context failed: {result:?} / {:?}",
                completion.future_result
            );
            self.status.state = ANCHOR_UNSUPPORTED;
            return;
        }
        self.context = completion.spatial_context;
        self.phase = Phase::Ready;
        // Back from unsupported (permission granted late)
        if self.status.state == ANCHOR_UNSUPPORTED {
            self.status.state = if self.status.uuid != [0; 16] {
                ANCHOR_LOST
            } else {
                ANCHOR_NONE
            };
        }
        info!("Anchor: spatial and persistence contexts ready");
    }

    fn poll_ready(&mut self, stage: &xr::Space, time: xr::Time) {
        match self.pending {
            Pending::None => (),
            Pending::Discover(future) => {
                if self.future_ready(future) {
                    self.pending = Pending::None;
                    self.finish_discovery(future, stage, time);
                }
                return;
            }
            Pending::Persist(future, entity_id, entity, replaced) => {
                if self.future_ready(future) {
                    self.pending = Pending::None;
                    self.finish_persist(future, entity_id, entity, replaced);
                }
                return;
            }
            Pending::Unpersist(future) => {
                if self.future_ready(future) {
                    self.pending = Pending::None;
                    let mut completion =
                        sys::UnpersistSpatialEntityCompletionEXT::out(ptr::null_mut());
                    let result = unsafe {
                        (self.operations_fns.unpersist_spatial_entity_complete)(
                            self.persistence_context,
                            future,
                            completion.as_mut_ptr(),
                        )
                    };
                    let completion = unsafe { completion.assume_init() };
                    info!(
                        "Anchor: unpersisted ({result:?} / {:?} / {:?})",
                        completion.future_result, completion.unpersist_result
                    );
                }
                return;
            }
        }

        if let Some(command) = self.command.take() {
            match command {
                AnchorCommand::Create(pose) => self.create(pose, stage, time),
                AnchorCommand::Forget => self.forget(),
            }
            return;
        }

        if self.entity.is_none() {
            // A persisted anchor to load (at start, or retried while it cannot be found)
            if self.status.uuid != [0; 16]
                && self
                    .last_discovery
                    .is_none_or(|instant| instant.elapsed() >= DISCOVERY_RETRY_INTERVAL)
            {
                self.start_discovery();
            }
            return;
        }

        if self
            .last_locate
            .is_none_or(|instant| instant.elapsed() >= LOCATE_INTERVAL)
        {
            self.last_locate = Some(Instant::now());
            self.locate(stage, time);
        }
    }

    fn start_discovery(&mut self) {
        self.last_discovery = Some(Instant::now());
        let uuid = sys::Uuid {
            data: self.status.uuid,
        };
        let filter = sys::SpatialDiscoveryPersistenceUuidFilterEXT {
            ty: sys::SpatialDiscoveryPersistenceUuidFilterEXT::TYPE,
            next: ptr::null(),
            persisted_uuid_count: 1,
            persisted_uuids: &raw const uuid,
        };
        let components = [
            sys::SpatialComponentTypeEXT::ANCHOR,
            sys::SpatialComponentTypeEXT::PERSISTENCE,
        ];
        let create_info = sys::SpatialDiscoverySnapshotCreateInfoEXT {
            ty: sys::SpatialDiscoverySnapshotCreateInfoEXT::TYPE,
            next: ptr::from_ref(&filter).cast(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
        let mut future = sys::FutureEXT::from_raw(0);
        let result = unsafe {
            (self.entity_fns.create_spatial_discovery_snapshot_async)(
                self.context,
                &raw const create_info,
                &raw mut future,
            )
        };
        if result == sys::Result::SUCCESS {
            self.pending = Pending::Discover(future);
        } else {
            self.fail("xrCreateSpatialDiscoverySnapshotAsyncEXT", result);
        }
    }

    fn finish_discovery(&mut self, future: sys::FutureEXT, stage: &xr::Space, time: xr::Time) {
        let completion_info = sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT {
            ty: sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE,
            next: ptr::null(),
            base_space: stage.as_raw(),
            time,
            future,
        };
        let mut completion = sys::CreateSpatialDiscoverySnapshotCompletionEXT::out(ptr::null_mut());
        let result = unsafe {
            (self.entity_fns.create_spatial_discovery_snapshot_complete)(
                self.context,
                &raw const completion_info,
                completion.as_mut_ptr(),
            )
        };
        let completion = unsafe { completion.assume_init() };
        if result != sys::Result::SUCCESS || completion.future_result != sys::Result::SUCCESS {
            self.fail(
                "discovery snapshot",
                if result == sys::Result::SUCCESS {
                    completion.future_result
                } else {
                    result
                },
            );
            return;
        }
        match self.query_anchor(completion.snapshot) {
            Some((entity_id, tracking, pose)) => {
                let mut entity = sys::SpatialEntityEXT::NULL;
                let create_info = sys::SpatialEntityFromIdCreateInfoEXT {
                    ty: sys::SpatialEntityFromIdCreateInfoEXT::TYPE,
                    next: ptr::null(),
                    entity_id,
                };
                let result = unsafe {
                    (self.entity_fns.create_spatial_entity_from_id)(
                        self.context,
                        &raw const create_info,
                        &raw mut entity,
                    )
                };
                if result != sys::Result::SUCCESS {
                    self.fail("xrCreateSpatialEntityFromIdEXT", result);
                    return;
                }
                self.entity = Some((entity_id, entity));
                self.apply_location(tracking, pose);
                info!(
                    "Anchor: persisted anchor {} found, {:?}",
                    uuid_hex(&self.status.uuid),
                    self.status.pose.map(|p| p.position)
                );
            }
            None => {
                if self.status.state != ANCHOR_LOST {
                    info!(
                        "Anchor: persisted anchor {} not found (yet)",
                        uuid_hex(&self.status.uuid)
                    );
                }
                self.status.state = ANCHOR_LOST;
            }
        }
    }

    // The first anchor entity of a snapshot: (id, tracking state, pose in the snapshot's base space)
    fn query_anchor(
        &mut self,
        snapshot: sys::SpatialSnapshotEXT,
    ) -> Option<(
        sys::SpatialEntityIdEXT,
        sys::SpatialEntityTrackingStateEXT,
        Pose,
    )> {
        let found = self.query_snapshot(snapshot);
        unsafe { (self.entity_fns.destroy_spatial_snapshot)(snapshot) };
        found
    }

    // Two-call idiom: entity count first, then the data with exactly sized arrays
    fn query_snapshot(
        &mut self,
        snapshot: sys::SpatialSnapshotEXT,
    ) -> Option<(
        sys::SpatialEntityIdEXT,
        sys::SpatialEntityTrackingStateEXT,
        Pose,
    )> {
        let components = [sys::SpatialComponentTypeEXT::ANCHOR];
        let condition = sys::SpatialComponentDataQueryConditionEXT {
            ty: sys::SpatialComponentDataQueryConditionEXT::TYPE,
            next: ptr::null(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
        let mut query = sys::SpatialComponentDataQueryResultEXT {
            ty: sys::SpatialComponentDataQueryResultEXT::TYPE,
            next: ptr::null_mut(),
            entity_id_capacity_input: 0,
            entity_id_count_output: 0,
            entity_ids: ptr::null_mut(),
            entity_state_capacity_input: 0,
            entity_state_count_output: 0,
            entity_states: ptr::null_mut(),
        };
        let result = unsafe {
            (self.entity_fns.query_spatial_component_data)(
                snapshot,
                &raw const condition,
                &raw mut query,
            )
        };
        let count = query.entity_id_count_output as usize;
        if result != sys::Result::SUCCESS || count == 0 {
            let outcome = format!("count query {result:?}, {count} entities");
            if outcome != self.last_query_outcome {
                info!("Anchor: {outcome}");
                self.last_query_outcome = outcome;
            }
            return None;
        }

        let mut ids = vec![sys::SpatialEntityIdEXT::from_raw(0); count];
        let mut states = vec![sys::SpatialEntityTrackingStateEXT::STOPPED; count];
        let mut locations = vec![xr::Posef::IDENTITY; count];
        let mut anchor_list = sys::SpatialComponentAnchorListEXT {
            ty: sys::SpatialComponentAnchorListEXT::TYPE,
            next: ptr::null_mut(),
            location_count: count as u32,
            locations: locations.as_mut_ptr(),
        };
        let mut query = sys::SpatialComponentDataQueryResultEXT {
            ty: sys::SpatialComponentDataQueryResultEXT::TYPE,
            next: ptr::from_mut(&mut anchor_list).cast(),
            entity_id_capacity_input: count as u32,
            entity_id_count_output: 0,
            entity_ids: ids.as_mut_ptr(),
            entity_state_capacity_input: count as u32,
            entity_state_count_output: 0,
            entity_states: states.as_mut_ptr(),
        };
        let result = unsafe {
            (self.entity_fns.query_spatial_component_data)(
                snapshot,
                &raw const condition,
                &raw mut query,
            )
        };
        let outcome = format!(
            "data query {result:?}, {} entities, first state {:?}",
            query.entity_id_count_output,
            states.first()
        );
        if outcome != self.last_query_outcome {
            info!("Anchor: {outcome}");
            self.last_query_outcome = outcome;
        }
        if result != sys::Result::SUCCESS || query.entity_id_count_output == 0 {
            return None;
        }
        Some((ids[0], states[0], crate::from_xr_pose(locations[0])))
    }

    fn apply_location(&mut self, tracking: sys::SpatialEntityTrackingStateEXT, pose: Pose) {
        if tracking == sys::SpatialEntityTrackingStateEXT::TRACKING {
            self.status.state = ANCHOR_LOCATED;
            self.status.pose = Some(pose);
        } else {
            // Last good pose stays
            self.status.state = ANCHOR_LOST;
        }
    }

    fn locate(&mut self, stage: &xr::Space, time: xr::Time) {
        let Some((_, entity)) = self.entity else {
            return;
        };
        let components = [sys::SpatialComponentTypeEXT::ANCHOR];
        let create_info = sys::SpatialUpdateSnapshotCreateInfoEXT {
            ty: sys::SpatialUpdateSnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            entity_count: 1,
            entities: &raw const entity,
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
            base_space: stage.as_raw(),
            time,
        };
        let mut snapshot = sys::SpatialSnapshotEXT::NULL;
        let result = unsafe {
            (self.entity_fns.create_spatial_update_snapshot)(
                self.context,
                &raw const create_info,
                &raw mut snapshot,
            )
        };
        let outcome = if result != sys::Result::SUCCESS {
            self.status.state = ANCHOR_LOST;
            format!("update snapshot failed: {result:?}")
        } else {
            match self.query_anchor(snapshot) {
                Some((_, tracking, pose)) => {
                    self.apply_location(tracking, pose);
                    format!("tracking state {tracking:?}")
                }
                None => {
                    self.status.state = ANCHOR_LOST;
                    "no anchor in the update snapshot".into()
                }
            }
        };
        if outcome != self.last_locate_outcome {
            info!("Anchor: locate: {outcome}");
            self.last_locate_outcome = outcome;
        }
    }

    fn create(&mut self, pose: Pose, stage: &xr::Space, time: xr::Time) {
        let create_info = sys::SpatialAnchorCreateInfoEXT {
            ty: sys::SpatialAnchorCreateInfoEXT::TYPE,
            next: ptr::null(),
            base_space: stage.as_raw(),
            time,
            pose: crate::to_xr_pose(pose),
        };
        let mut entity_id = sys::SpatialEntityIdEXT::from_raw(0);
        let mut entity = sys::SpatialEntityEXT::NULL;
        let result = unsafe {
            (self.anchor_fns.create_spatial_anchor)(
                self.context,
                &raw const create_info,
                &raw mut entity_id,
                &raw mut entity,
            )
        };
        if result != sys::Result::SUCCESS {
            self.fail("xrCreateSpatialAnchorEXT", result);
            return;
        }
        let persist_info = sys::SpatialEntityPersistInfoEXT {
            ty: sys::SpatialEntityPersistInfoEXT::TYPE,
            next: ptr::null(),
            spatial_context: self.context,
            spatial_entity_id: entity_id,
        };
        let mut future = sys::FutureEXT::from_raw(0);
        let result = unsafe {
            (self.operations_fns.persist_spatial_entity_async)(
                self.persistence_context,
                &raw const persist_info,
                &raw mut future,
            )
        };
        if result != sys::Result::SUCCESS {
            unsafe { (self.entity_fns.destroy_spatial_entity)(entity) };
            self.fail("xrPersistSpatialEntityAsyncEXT", result);
            return;
        }
        let replaced = (self.status.uuid != [0; 16]).then_some(self.status.uuid);
        self.status.state = ANCHOR_CREATING;
        self.pending = Pending::Persist(future, entity_id, entity, replaced);
        info!("Anchor: creating at {:?}", pose.position);
    }

    fn finish_persist(
        &mut self,
        future: sys::FutureEXT,
        entity_id: sys::SpatialEntityIdEXT,
        entity: sys::SpatialEntityEXT,
        replaced: Option<[u8; 16]>,
    ) {
        let mut completion = sys::PersistSpatialEntityCompletionEXT::out(ptr::null_mut());
        let result = unsafe {
            (self.operations_fns.persist_spatial_entity_complete)(
                self.persistence_context,
                future,
                completion.as_mut_ptr(),
            )
        };
        let completion = unsafe { completion.assume_init() };
        if result != sys::Result::SUCCESS || completion.future_result != sys::Result::SUCCESS {
            unsafe { (self.entity_fns.destroy_spatial_entity)(entity) };
            warn!(
                "Anchor: persist failed: {result:?} / {:?} / {:?}",
                completion.future_result, completion.persist_result
            );
            self.status.state = ANCHOR_ERROR;
            return;
        }

        if let Some((_, old)) = self.entity.replace((entity_id, entity)) {
            unsafe { (self.entity_fns.destroy_spatial_entity)(old) };
        }
        self.status.uuid = completion.persist_uuid.data;
        self.status.generation = self.status.generation.wrapping_add(1);
        self.status.state = ANCHOR_LOST; // until the first locate
        self.last_locate = None;
        info!(
            "Anchor: created and persisted as {} ({:?})",
            uuid_hex(&self.status.uuid),
            completion.persist_result
        );

        if let Some(old_uuid) = replaced.filter(|old| *old != self.status.uuid) {
            self.unpersist(old_uuid);
        }
    }

    fn unpersist(&mut self, uuid: [u8; 16]) {
        let info = sys::SpatialEntityUnpersistInfoEXT {
            ty: sys::SpatialEntityUnpersistInfoEXT::TYPE,
            next: ptr::null(),
            persist_uuid: sys::Uuid { data: uuid },
        };
        let mut future = sys::FutureEXT::from_raw(0);
        let result = unsafe {
            (self.operations_fns.unpersist_spatial_entity_async)(
                self.persistence_context,
                &raw const info,
                &raw mut future,
            )
        };
        if result == sys::Result::SUCCESS {
            self.pending = Pending::Unpersist(future);
        } else {
            warn!("Anchor: xrUnpersistSpatialEntityAsyncEXT failed: {result:?}");
        }
    }

    fn forget(&mut self) {
        if let Some((_, entity)) = self.entity.take() {
            unsafe { (self.entity_fns.destroy_spatial_entity)(entity) };
        }
        let uuid = self.status.uuid;
        self.status = AnchorStatus {
            state: ANCHOR_NONE,
            uuid: [0; 16],
            pose: None,
            generation: self.status.generation.wrapping_add(1),
        };
        if uuid != [0; 16] {
            self.unpersist(uuid);
        }
        info!("Anchor: forgotten");
    }
}

impl Drop for PersistentAnchor {
    fn drop(&mut self) {
        unsafe {
            if let Some((_, entity)) = self.entity.take() {
                (self.entity_fns.destroy_spatial_entity)(entity);
            }
            if self.context != sys::SpatialContextEXT::NULL {
                (self.entity_fns.destroy_spatial_context)(self.context);
            }
            if self.persistence_context != sys::SpatialPersistenceContextEXT::NULL {
                (self.persistence_fns.destroy_spatial_persistence_context)(
                    self.persistence_context,
                );
            }
        }
    }
}
