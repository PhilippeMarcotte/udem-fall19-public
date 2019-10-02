import numpy as np
def vel2wheel(v, omega, wheel_dist, wheel_rad):
    
    gain = 1
    trim = 0
    
    # Maximal speed
    if v > 0.5:
        v = 0.5
    elif v < -0.5:
        v = -0.5
    
    
##### Fill the code here:
    v = v / (wheel_rad * np.pi * 2)
    omega = omega / (wheel_rad * np.pi * 2)
    left_rate = (v - (wheel_dist / 2) * omega) 
    right_rate = (v + (wheel_dist / 2) * omega)
    
####


    
    
    return left_rate, right_rate