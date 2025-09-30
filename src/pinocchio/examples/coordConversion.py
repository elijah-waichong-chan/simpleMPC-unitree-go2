# quaternion in xyzw <-> rpy helpers
import numpy as np
import pinocchio as pin

def euler_to_quad(roll, pitch, yaw):
    cr,sr = np.cos(roll/2), np.sin(roll/2)
    cp,sp = np.cos(pitch/2), np.sin(pitch/2)
    cy,sy = np.cos(yaw/2), np.sin(yaw/2)
    
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    return np.array([qx,qy,qz,qw], float)

def quad_to_euler(qx, qy, qz, qw):

    # Make sure qx^2 + qy^2 + qz^2 + qw^2 == 1
    length = np.linalg.norm([qx, qy, qz, qw])
    qx, qy, qz, qw = qx/length, qy/length, qz/length, qw/length

    q_eig = pin.Quaternion(qw, qx, qy, qz)
    R = q_eig.toRotationMatrix()
    rpy = pin.rpy.matrixToRpy(R)

    return float(rpy[0]), float(rpy[1]), float(rpy[2])
