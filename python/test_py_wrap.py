import numpy as np
import pyoctomap

pts = np.random.uniform(size=(10000, 3))
k = pyoctomap.Pointcloud(pts)
print(f"np.allclose(k.numpy(), pts): {np.allclose(k.numpy(), pts)}")

v = pyoctomap.Vector3(np.arange(3, dtype=np.float32))
print(f"np.allclose(v.numpy(), np.arange(3)): {np.allclose(v.numpy(), np.arange(3))}")

