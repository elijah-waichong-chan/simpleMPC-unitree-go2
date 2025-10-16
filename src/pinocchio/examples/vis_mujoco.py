import numpy as np
import mujoco
import mujoco.viewer
from pathlib import Path

# Set this to your MJCF file directory
repo = Path(__file__).resolve().parents[3]
scene_path = repo / "unitree_mujoco" / "unitree_robots" / "go2" / "scene.xml"


# Load the MuJoCo model
m = mujoco.MjModel.from_xml_path(str(scene_path))
d = mujoco.MjData(m)

# Helper function to convert between pinocchio and mujoco coordinates
def pin_to_mj_qpos(m, pin_model, q_pin):
    """
    Map a Pinocchio configuration q_pin into MuJoCo's d.qpos order.
    Handles free-flyer (if present) and 1-DoF joints by name.
    """
    qpos = np.zeros(m.nq)

    # --- 1) Handle free-flyer (if the first Pin joint is free) ---
    has_free = False
    if pin_model.joints[1].nq == 7:  # typical free-flyer at root in Pin
        has_free = True
        px, py, pz, qx, qy, qz, qw = q_pin[:7]
        # MuJoCo wants [px, py, pz, qw, qx, qy, qz]
        qpos[:7] = [px, py, pz, qw, qx, qy, qz]

    # --- 2) Map 1-DoF joints by name ---
    # Pin indexing helpers
    # Pin joint 'i' occupies q segment [idx_q : idx_q + nq]
    for i in range(1, len(pin_model.joints)):
        jpin = pin_model.joints[i]
        name = pin_model.names[i]
        nq_i = jpin.nq
        idx_q = jpin.idx_q

        # skip free-flyer chunk (already copied)
        if has_free and i == 1:
            continue

        # Only handle 1-DoF here (revolute/prismatic); extend as needed for spherical etc.
        if nq_i == 1:
            # Find same-named MuJoCo joint; set its qpos slot
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid != -1:
                mj_qadr = m.jnt_qposadr[jid]  # starting index in qpos
                qpos[mj_qadr] = q_pin[idx_q]
            else:
                # If names differ, you can add a name remapping dict here.
                pass       

    return qpos

# visualization call
def visualize(pin_model, pin_q):
    d.qpos[:] = pin_to_mj_qpos(m, pin_model, pin_q)
    print(d.qpos) 
    mujoco.mj_forward(m, d)   # recompute derived quantities

    # View it (no stepping needed if you just want to look at the pose)
    with mujoco.viewer.launch_passive(m, d) as v:
        v.sync()
        # Keep window open until closed
        while v.is_running():
            v.sync()