# Converts Pico's SM_Hand_{L,R}.FBX into a SteamVR rendermodel OBJ expressed in the ALVR
# hand-tracker device frame (OpenXR palm joint * left/right_hand_tracking_*_offset).
#
# The FBX files are the hand meshes of the PICO OpenXR Unreal plugin
# (Content/PICOOpenXR/Meshes/Hand), exported as ASCII FBX; their skeleton bones are named after the
# XR_EXT_hand_tracking joints, which gives the palm joint frame. Requires Blender 4.5+ (wm.fbx_import
# reads ASCII FBX). Usage, from the repo root:
#   blender -b --factory-startup --python resources/pico_hand_to_rendermodel.py -- #     resources/SM_Hand_L.FBX left #     alvr/xtask/resources/alvr_server/resources/rendermodels/pico_hand_left/pico_hand_left.obj #     pico_hand_left #     alvr/xtask/resources/alvr_server/resources/rendermodels/pico_hand_left/pico_hand_albedo.png
# (same with SM_Hand_R.FBX / right). The mesh only lines up with the default hand tracking offsets
# in settings.rs; regenerate it if those defaults change.
import bpy, sys, math
from mathutils import Vector, Matrix, Quaternion, Euler
argv = sys.argv[sys.argv.index("--")+1:]
fbx, side, out_obj, mtl_name = argv[0], argv[1], argv[2], argv[3]
out_png = argv[4] if len(argv) > 4 else None
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.fbx_import(filepath=fbx)
arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
mesh_obj = next(o for o in bpy.data.objects if o.type == 'MESH')
arm.data.pose_position = 'REST'
bpy.context.view_layer.update()

# Pico's DiffuseUV is a planar projection: palm and back share texels, which makes baked shading
# impossible. Unwrap into a new, non-overlapping UV layer used for both the OBJ and the bake.
for o in bpy.context.view_layer.objects: o.select_set(False)
mesh_obj.select_set(True)
bpy.context.view_layer.objects.active = mesh_obj
mesh_obj.data.uv_layers.active = mesh_obj.data.uv_layers.new(name="BakeUV")
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.uv.smart_project(angle_limit=math.radians(66), island_margin=0.004)
bpy.ops.object.mode_set(mode='OBJECT')
p = side  # 'left' / 'right'
def head(name): return arm.matrix_world @ arm.data.bones[f"{p}_{name}"].head_local

palm = head("palm")
fwd = (head("middle_proximal") - head("middle_metacarpal")).normalized()   # toward fingertips = -Z_palm
across = head("index_proximal") - head("little_proximal")
dorsal = across.cross(fwd) if p == 'left' else -across.cross(fwd)        # +Y_palm (back of hand)
z = -fwd
y = (dorsal - dorsal.dot(z) * z).normalized()
x = y.cross(z)
palm_to_world = Matrix((
    (x[0], y[0], z[0], palm[0]),
    (x[1], y[1], z[1], palm[1]),
    (x[2], y[2], z[2], palm[2]),
    (0, 0, 0, 1)))
assert abs(palm_to_world.to_3x3().determinant() - 1) < 1e-4

# ALVR default offsets (settings.rs), mirrored for the right hand like get_hand_skeleton_offsets()
t = Vector((0.04, -0.02, -0.13)); r = [0.0, -45.0, -90.0]
if p == 'right':
    t = Vector((-t.x, t.y, t.z)); r = [r[0], -r[1], -r[2]]
# glam Quat::from_euler(EulerRot::XYZ, a, b, c) == Rx(a) * Ry(b) * Rz(c)
q = (Quaternion((1, 0, 0), math.radians(r[0])) @ Quaternion((0, 1, 0), math.radians(r[1]))
     @ Quaternion((0, 0, 1), math.radians(r[2])))
# device pose in palm frame: rotation q, origin q*t
device_in_palm = Matrix.Translation(q @ t) @ q.to_matrix().to_4x4()
world_to_device = device_in_palm.inverted() @ palm_to_world.inverted()

dg = bpy.context.evaluated_depsgraph_get()
ev = mesh_obj.evaluated_get(dg)
me = ev.to_mesh()
me.calc_loop_triangles()
M = world_to_device @ mesh_obj.matrix_world
N = M.to_3x3().inverted().transposed()
verts = [M @ v.co for v in me.vertices]
uvl = me.uv_layers.active.data
cn = me.corner_normals

# winding check: signed volume must be positive (outward-facing CCW triangles)
vol = 0.0
for tri in me.loop_triangles:
    a, b, c = (verts[i] for i in tri.vertices)
    vol += a.dot(b.cross(c)) / 6
flip = vol < 0
mn = [min(v[i] for v in verts) for i in range(3)]; mx = [max(v[i] for v in verts) for i in range(3)]
print(f"RESULT side={p} tris={len(me.loop_triangles)} volume={vol*1e6:.1f}cm3 flip={flip} "
      f"bbox_min={[round(c,3) for c in mn]} bbox_max={[round(c,3) for c in mx]}")

with open(out_obj, "w", newline="\n") as f:
    f.write(f"# Pico {p} hand (SM_Hand from the PICO OpenXR Unreal plugin), ALVR hand-tracker device frame\n")
    f.write(f"mtllib {mtl_name}.mtl\no pico_hand_{p}\n")
    for v in verts: f.write(f"v {v.x:.6f} {v.y:.6f} {v.z:.6f}\n")
    vt_index = {}; vts = []; vns = []; faces = []
    for tri in me.loop_triangles:
        idx = []
        for li, vi in zip(tri.loops, tri.vertices):
            uv = tuple(round(c, 6) for c in uvl[li].uv)
            if uv not in vt_index: vt_index[uv] = len(vts) + 1; vts.append(uv)
            n = (N @ Vector(cn[li].vector)).normalized()
            if flip: n = -n
            vns.append(n)
            idx.append((vi + 1, vt_index[uv], len(vns)))
        faces.append(idx[::-1] if flip else idx)
    for uv in vts: f.write(f"vt {uv[0]:.6f} {uv[1]:.6f}\n")
    for n in vns: f.write(f"vn {n.x:.4f} {n.y:.4f} {n.z:.4f}\n")
    f.write(f"usemtl {mtl_name}\ns 1\n")
    for face in faces: f.write("f " + " ".join(f"{a}/{b}/{c}" for a, b, c in face) + "\n")


# SteamVR draws rendermodels unlit, so shading is baked into the albedo (like the controller
# textures): Pico's hand look (dark InnerColor body, light OutColor fresnel rim, from MI_Hand)
# approximated view-independently with a rim on surfaces that face sideways relative to the back
# of the hand, plus ambient occlusion between the fingers.
if out_png:
    INNER = (0.1311, 0.1543, 0.2448)
    RIM = (0.7101, 0.7946, 1.0)
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 64
    scene.view_settings.view_transform = 'Standard'
    img = bpy.data.images.new("hand_bake", 1024, 1024, alpha=False)
    img.colorspace_settings.name = 'sRGB'
    dorsal_obj = (mesh_obj.matrix_world.to_3x3().inverted() @ y).normalized()

    mat = bpy.data.materials.new("bake")
    mat.use_nodes = True
    nt = mat.node_tree; nodes = nt.nodes; links = nt.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    emit = nodes.new("ShaderNodeEmission")
    links.new(emit.outputs[0], out.inputs[0])
    texco = nodes.new("ShaderNodeTexCoord")
    dot = nodes.new("ShaderNodeVectorMath"); dot.operation = 'DOT_PRODUCT'
    dot.inputs[1].default_value = dorsal_obj
    links.new(texco.outputs["Normal"], dot.inputs[0])
    absn = nodes.new("ShaderNodeMath"); absn.operation = 'ABSOLUTE'
    links.new(dot.outputs["Value"], absn.inputs[0])
    inv = nodes.new("ShaderNodeMath"); inv.operation = 'SUBTRACT'; inv.inputs[0].default_value = 1.0
    links.new(absn.outputs[0], inv.inputs[1])
    pw = nodes.new("ShaderNodeMath"); pw.operation = 'POWER'; pw.inputs[1].default_value = 4.0
    links.new(inv.outputs[0], pw.inputs[0])
    # Back of the hand slightly brighter than the palm: a soft key light from the dorsal side
    key = nodes.new("ShaderNodeMath"); key.operation = 'MULTIPLY_ADD'
    key.inputs[1].default_value = 0.25; key.inputs[2].default_value = 1.0
    links.new(dot.outputs["Value"], key.inputs[0])
    body = nodes.new("ShaderNodeMix"); body.data_type = 'RGBA'; body.blend_type = 'MULTIPLY'
    body.inputs["Factor"].default_value = 1.0
    body.inputs[6].default_value = (*INNER, 1)
    comb = nodes.new("ShaderNodeCombineXYZ")
    for i in range(3): links.new(key.outputs[0], comb.inputs[i])
    links.new(comb.outputs[0], body.inputs[7])
    rim = nodes.new("ShaderNodeMix"); rim.data_type = 'RGBA'
    rimf = nodes.new("ShaderNodeMath"); rimf.operation = 'MULTIPLY'; rimf.inputs[1].default_value = 0.75
    links.new(pw.outputs[0], rimf.inputs[0])
    links.new(rimf.outputs[0], rim.inputs["Factor"])
    links.new(body.outputs[2], rim.inputs[6])
    rim.inputs[7].default_value = (*RIM, 1)
    ao = nodes.new("ShaderNodeAmbientOcclusion"); ao.samples = 16; ao.inputs["Distance"].default_value = 0.03
    aom = nodes.new("ShaderNodeMapRange")
    aom.inputs["To Min"].default_value = 0.45
    links.new(ao.outputs["AO"], aom.inputs["Value"])
    shade = nodes.new("ShaderNodeMix"); shade.data_type = 'RGBA'; shade.blend_type = 'MULTIPLY'
    shade.inputs["Factor"].default_value = 1.0
    links.new(rim.outputs[2], shade.inputs[6])
    aoc = nodes.new("ShaderNodeCombineXYZ")
    for i in range(3): links.new(aom.outputs[0], aoc.inputs[i])
    links.new(aoc.outputs[0], shade.inputs[7])
    links.new(shade.outputs[2], emit.inputs["Color"])
    tex = nodes.new("ShaderNodeTexImage"); tex.image = img
    nodes.active = tex

    mesh_obj.data.materials.clear()
    mesh_obj.data.materials.append(mat)
    mesh_obj.data.uv_layers.active = mesh_obj.data.uv_layers["BakeUV"]
    for o in bpy.context.view_layer.objects: o.select_set(False)
    mesh_obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_obj
    scene.render.bake.margin = 16
    bpy.ops.object.bake(type='EMIT')
    img.filepath_raw = out_png
    img.file_format = 'PNG'
    img.save()
    print("BAKED", out_png)
