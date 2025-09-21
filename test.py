from scipy.spatial.transform import Rotation as R
import numpy as np
#   x: 0.3259107076471432
#   y: -0.945108301613783
#   z: -0.02268808313215904
#   w: -0.006144895884968449

# eular_r = R.from_quat([-0.6756383830565972, 0.6848048481418338, 0.1755965679418997, 0.2090955299118864]).as_euler('xyz', degrees=False)
eular_r = R.from_quat([0.3259107076471432, -0.945108301613783, -0.02268808313215904, -0.006144895884968449]).as_euler('xyz', degrees=False)
# eular_r[1] += np.pi/2 
print(eular_r)