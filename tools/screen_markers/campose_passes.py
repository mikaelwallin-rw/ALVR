import sys, numpy as np
sys.argv = [sys.argv[0]] + sys.argv[1:]
import importlib.util
spec = importlib.util.spec_from_file_location('ca', sys.argv[1]); ca = importlib.util.module_from_spec(spec); spec.loader.exec_module(ca)
for line in open(sys.argv[2], encoding='utf-8', errors='replace'):
    m = ca.LINE.search(line)
    if not m: continue
    g = [float(x) for x in m.groups()[1:]]
    A, L, B = ca.pose(g[1:8]), ca.pose(g[8:15]), ca.pose(g[15:22])
    C = np.linalg.inv(A) @ L @ B
    print(line[6:18], (m.group(1) or 'Left')[:1], 'cam_in_view_cm', np.round(C[:3,3]*100, 1), 'head_y', round(A[1,3], 3))
