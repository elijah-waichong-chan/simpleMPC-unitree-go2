import pinocchio as pin 
import numpy as np

from readURDF import createFloatingBaseModel

model, data, vmodel, vdata, cmodel, cdata  = createFloatingBaseModel()

q = pin.neutral(model)
v = np.zeros(model.nv)

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
pin.ccrba(model, data, q, v) 

# Compute the necessary model constants

m = data.Ig.mass
lever = data.Ig.lever
I_com = data.Ig.inertia
g = 9.81

print("Robot Mass(kg):", m)
print("I_CoM:", I_com)