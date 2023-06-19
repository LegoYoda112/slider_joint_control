import ankle.InverseKinematics
import ankle.ForwardKinematics
import numpy as np

import time



start = time.time()
ankle.ForwardKinematics.fk(-4.0 * (np.pi / 180.0), 0.0 * (np.pi / 180.0))
end = time.time()

print((end - start) * 1e+9)