# Converts Pico's SM_Hand_{L,R}.FBX (PICO OpenXR Unreal plugin hand meshes) into the skinned hand
# mesh drawn natively by the ALVR client (alvr/graphics/src/hands.rs).
#
# The FBX bones are named after the XR_EXT_hand_tracking joints, and PICO's own hand component
# drives them by setting each bone's world rotation to the tracked joint rotation, so the bind pose
# bone frames are the OpenXR joint frames (+Y out of the back of the hand, -Z toward the
# fingertips) up to one constant axis change introduced by the FBX import. That change is found
# here by testing the 24 axis-aligned rotations against the skeleton geometry, and every joint must
# agree with it.
#
# Requires Blender 4.5+ (ASCII FBX import). Usage, from the repo root:
#   blender -b --factory-startup --python resources/pico_hand_to_skinned_mesh.py -- \
#     resources/SM_Hand_L.FBX left alvr/graphics/resources/hands/pico_hand_left.bin
# (same with SM_Hand_R.FBX / right).
#
# Output (little endian):
#   b"AHND", u32 version (1), u32 vertex count, u32 index count
#   26 x mat4 f32 column major: joint_from_mesh (inverse bind) per XR_EXT_hand_tracking joint
#   vertices: position f32x3 (m), normal f32x3, joint indices u8x4, weights unorm8x4 (sum 255)
#   indices: u16 triangle list, padded to 4 bytes
import bpy, sys, struct, itertools
from mathutils import Matrix, Vector

JOINTS = [
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
]
# Finger joints with a child joint (index + 1) in the same chain
CHAIN_JOINTS = [j for j in range(2, 26) if not JOINTS[j].endswith("tip")]
THUMB = {2, 3, 4, 5}
MIN_AGREEMENT = 0.9

argv = sys.argv[sys.argv.index("--") + 1:]
fbx, side, out_bin = argv[0], argv[1], argv[2]

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.fbx_import(filepath=fbx)
arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
mesh_obj = next(o for o in bpy.data.objects if o.type == 'MESH')
arm.data.pose_position = 'REST'
bpy.context.view_layer.update()


def bone_frame(name):
    m = arm.matrix_world @ arm.data.bones[f"{side}_{name}"].matrix_local
    return m.to_translation(), m.to_quaternion().to_matrix()  # strips the import scale


frames = [bone_frame(j) for j in JOINTS]
heads = [f[0] for f in frames]

fwd = (heads[12] - heads[11]).normalized()  # middle metacarpal -> proximal
across = heads[7] - heads[22]  # little proximal -> index proximal
dorsal = across.cross(fwd) if side == 'left' else -across.cross(fwd)
dorsal = (dorsal - dorsal.dot(fwd) * fwd).normalized()

# All 24 proper axis-aligned rotations
candidates = []
for perm in itertools.permutations(range(3)):
    for signs in itertools.product((1, -1), repeat=3):
        m = Matrix.Diagonal((0, 0, 0)).to_3x3()
        for row, (col, sign) in enumerate(zip(perm, signs)):
            m[row][col] = sign
        if abs(m.determinant() - 1) < 1e-6:
            candidates.append(m)


def agreement(correction):
    z_scores, y_scores = [], []
    for j in CHAIN_JOINTS:
        r = frames[j][1] @ correction
        to_child = (heads[j + 1] - heads[j]).normalized()
        z_scores.append(-r.col[2].dot(to_child))
        if j not in THUMB:
            y_scores.append(r.col[1].dot(dorsal))
    return min(z_scores), min(y_scores), sum(z_scores) / len(z_scores), sum(y_scores) / len(y_scores)


best = max(candidates, key=lambda c: sum(agreement(c)[2:]))
z_min, y_min, z_mean, y_mean = agreement(best)
print(f"RESULT side={side} correction={[list(r) for r in best]} "
      f"z(min {z_min:.3f} mean {z_mean:.3f}) y(min {y_min:.3f} mean {y_mean:.3f})")
assert z_min > MIN_AGREEMENT and y_min > MIN_AGREEMENT, "bone frames do not match the OpenXR joints"

inverse_binds = []
for head, rot in frames:
    rest = Matrix.Translation(head) @ (rot @ best).to_4x4()
    inverse_binds.append(rest.inverted())

dg = bpy.context.evaluated_depsgraph_get()
me = mesh_obj.evaluated_get(dg).to_mesh()
me.calc_loop_triangles()
to_world = mesh_obj.matrix_world
normal_matrix = to_world.to_3x3().inverted().transposed()
group_joint = {}
for group in mesh_obj.vertex_groups:
    name = group.name.removeprefix(f"{side}_")
    if name in JOINTS:
        group_joint[group.index] = JOINTS.index(name)

vertex_data = bytearray()
unweighted = 0
for v in me.vertices:
    position = to_world @ v.co
    normal = (normal_matrix @ v.normal).normalized()
    weights = sorted(
        ((g.weight, group_joint[g.group]) for g in v.groups if g.group in group_joint and g.weight > 0),
        reverse=True)[:4]
    if not weights:
        weights = [(1.0, 0)]  # palm
        unweighted += 1
    total = sum(w for w, _ in weights)
    quantized = [round(w / total * 255) for w, _ in weights]
    quantized[0] += 255 - sum(quantized)
    joints = [j for _, j in weights]
    while len(joints) < 4:
        joints.append(0)
        quantized.append(0)
    vertex_data += struct.pack("<3f3f4B4B", *position, *normal, *joints, *quantized)

indices = [i for tri in me.loop_triangles for i in tri.vertices]
assert len(me.vertices) < 65536

volume = 0.0
for tri in me.loop_triangles:
    a, b, c = (to_world @ me.vertices[i].co for i in tri.vertices)
    volume += a.dot(b.cross(c)) / 6
positions = [to_world @ v.co for v in me.vertices]
lo = [round(min(p[i] for p in positions), 3) for i in range(3)]
hi = [round(max(p[i] for p in positions), 3) for i in range(3)]
print(f"RESULT side={side} vertices={len(me.vertices)} triangles={len(me.loop_triangles)} "
      f"unweighted={unweighted} volume={volume * 1e6:.1f}cm3 bbox={lo}..{hi}")

with open(out_bin, "wb") as f:
    f.write(b"AHND")
    f.write(struct.pack("<3I", 1, len(me.vertices), len(indices)))
    for m in inverse_binds:
        f.write(struct.pack("<16f", *(m[row][col] for col in range(4) for row in range(4))))
    f.write(vertex_data)
    f.write(struct.pack(f"<{len(indices)}H", *indices))
    if len(indices) % 2:
        f.write(b"\0\0")
print("WROTE", out_bin)
