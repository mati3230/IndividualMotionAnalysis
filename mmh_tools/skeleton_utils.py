import numpy as np

skeleton =\
{
    "HIPS": {
        "SPINE": np.array([0, 0.5, 0], dtype=np.float32),
        "UPLEG_R": np.array([0.5, -0.5, 0], dtype=np.float32),
        "UPLEG_L": np.array([-0.5, -0.5, 0], dtype=np.float32)
    },
    "SPINE": {
        "CHEST": np.array([0, 0.5, 0], dtype=np.float32)
    },
    "CHEST": {
        "NECK": np.array([0, 0.5, 0], dtype=np.float32)
    },
    "NECK": {
        "HEAD": np.array([0, 0.5, 0], dtype=np.float32),
        "SHOULDER_R": np.array([0.5, 0, 0], dtype=np.float32),
        "SHOULDER_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "SHOULDER_R": {
        "ARM_R": np.array([0.5, 0, 0], dtype=np.float32)
    },
    "SHOULDER_L": {
        "ARM_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "ARM_R": {
        "FOREARM_R": np.array([0.5, 0, 0], dtype=np.float32)
    },
    "ARM_L": {
        "FOREARM_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "FOREARM_R": {
        "HAND_R": np.array([0.5, 0, 0], dtype=np.float32)
    },
    "FOREARM_L": {
        "HAND_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "UPLEG_R": {
        "LEG_R": np.array([0, -0.5, 0], dtype=np.float32)
    },
    "UPLEG_L": {
        "LEG_L": np.array([0, -0.5, 0], dtype=np.float32)
    },
    "LEG_R": {
        "FOOT_R": np.array([0, -0.5, 0], dtype=np.float32)
    },
    "LEG_L": {
        "FOOT_L": np.array([0, -0.5, 0], dtype=np.float32)
    },
    "FOOT_R": {
        "TOE_R": np.array([0, -0.25, -0.25], dtype=np.float32)
    },
    "FOOT_L": {
        "TOE_L": np.array([0, -0.25, -0.25], dtype=np.float32)
    }
}

child_to_parent = {
    "TOE_L": "FOOT_L",
    "FOOT_L": "LEG_L",
    "LEG_L": "UPLEG_L",
    "UPLEG_L": "HIPS",
    "TOE_R": "FOOT_R",
    "FOOT_R": "LEG_R",
    "LEG_R": "UPLEG_R",
    "UPLEG_R": "HIPS",
    "HAND_L": "FOREARM_L",
    "FOREARM_L": "ARM_L",
    "ARM_L": "SHOULDER_L",
    "SHOULDER_L": "NECK",
    "HAND_R": "FOREARM_R",
    "FOREARM_R": "ARM_R",
    "ARM_R": "SHOULDER_R",
    "SHOULDER_R": "NECK",
    "HEAD": "NECK",
    "NECK": "CHEST",
    "CHEST": "SPINE", 
    "SPINE": "HIPS"
}

skeleton_osim = {
    "TOE_L": {
        "FOOT_L": np.array([0, 0.25, 0.25], dtype=np.float32)
    },
    "FOOT_L": {
        "LEG_L": np.array([0, 0.5, 0], dtype=np.float32)
    },
    "LEG_L": {
        "UPLEG_L": np.array([0, 0.5, 0], dtype=np.float32)
    },
    "UPLEG_L": {
        "HIPS": np.array([0.5, 0.5, 0], dtype=np.float32)
    },
    "HIPS": {
        "UPLEG_R": np.array([0.5, -0.5, 0], dtype=np.float32),
        "SPINE": np.array([0, 0.5, 0], dtype=np.float32)
    },
    "SPINE": {
        "CHEST": np.array([0, 0.5, 0], dtype=np.float32)
    },
    "CHEST": {
        "NECK": np.array([0, 0.5, 0], dtype=np.float32)
    },
    "NECK": {
        "HEAD": np.array([0, 0.5, 0], dtype=np.float32),
        "SHOULDER_R": np.array([0.5, 0, 0], dtype=np.float32),
        "SHOULDER_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "SHOULDER_R": {
        "ARM_R": np.array([0.5, 0, 0], dtype=np.float32)
    },
    "SHOULDER_L": {
        "ARM_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "ARM_R": {
        "FOREARM_R": np.array([0.5, 0, 0], dtype=np.float32)
    },
    "ARM_L": {
        "FOREARM_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "FOREARM_R": {
        "HAND_R": np.array([0.5, 0, 0], dtype=np.float32)
    },
    "FOREARM_L": {
        "HAND_L": np.array([-0.5, 0, 0], dtype=np.float32)
    },
    "UPLEG_R": {
        "LEG_R": np.array([0, -0.5, 0], dtype=np.float32)
    },
    "LEG_R": {
        "FOOT_R": np.array([0, -0.5, 0], dtype=np.float32)
    },
    "FOOT_R": {
        "TOE_R": np.array([0, -0.25, -0.25], dtype=np.float32)
    }
}

osim_child_to_parent = {
    "FOOT_L": "TOE_L",
    "LEG_L": "FOOT_L",
    "UPLEG_L": "LEG_L",
    "HIPS": "UPLEG_L",
    "TOE_R": "FOOT_R",
    "FOOT_R": "LEG_R",
    "LEG_R": "UPLEG_R",
    "UPLEG_R": "HIPS",
    "HAND_L": "FOREARM_L",
    "FOREARM_L": "ARM_L",
    "ARM_L": "SHOULDER_L",
    "SHOULDER_L": "NECK",
    "HAND_R": "FOREARM_R",
    "FOREARM_R": "ARM_R",
    "ARM_R": "SHOULDER_R",
    "SHOULDER_R": "NECK",
    "HEAD": "NECK",
    "NECK": "CHEST",
    "CHEST": "SPINE", 
    "SPINE": "HIPS"
}

ax_x = np.array([1, 0, 0])
ax_y = np.array([0, 1, 0])
ax_z = np.array([0, 0, 1])

# body plane normals
sagittal = np.cross(ax_y, ax_z)
frontal = np.cross(ax_y, ax_x)
transversal = np.cross(ax_x, ax_z)

sag_idx = 1
trans_idx = 0
front_idx = 2

flex_vec = -np.array(ax_y, copy=True)
add_vec = np.array(ax_x, copy=True)


# skeleton from which we will calculate the angles
skeleton_angles =\
{
    # list of tuples where each tuple consist of a body plane and a default deviation vector
    "HIPS": [("pelvis_tilt", sagittal, -ax_y, 2, 1, sag_idx), ("pelvis_list", frontal, -ax_y, 0, 3, front_idx), ("pelvis_rotation", transversal, ax_z, 0, 2, trans_idx)],
    "UPLEG_R": [("hip_flexion_r", sagittal, -ax_y, 2, 1, sag_idx), ("hip_adduction_r", frontal, -ax_y, 0, 3, front_idx), ("hip_rotation_r", transversal, ax_z, 0, 2, trans_idx)],
    "UPLEG_L": [("hip_flexion_l", sagittal, -ax_y, 2, 1, sag_idx), ("hip_adduction_l", frontal, -ax_y, 0, 3, front_idx), ("hip_rotation_l", transversal, ax_z, 0, 2, trans_idx)],
    "LEG_R": [("knee_angle_r", sagittal, -ax_y, 2, 1, sag_idx)],
    "LEG_L": [("knee_angle_l", sagittal, -ax_y, 2, 1, sag_idx)],
    "FOOT_R": [("ankle_angle_r", sagittal, -ax_y, 2, 1, sag_idx)],
    "FOOT_L": [("ankle_angle_l", sagittal, -ax_y, 2, 1, sag_idx)],
    "SPINE": [("lumbar_extension", sagittal, -ax_y, 2, 1, sag_idx), ("lumbar_bending", frontal, -ax_y, 0, 3, front_idx), ("lumbar_rotation", transversal, ax_z, 0, 2, trans_idx)], 
    "ARM_R": [("arm_flex_r", sagittal, -ax_y, 2, 1, sag_idx), ("arm_add_r", frontal, -ax_y, 0, 3, front_idx), ("arm_rot_r", transversal, ax_z, 0, 2, trans_idx)],
    "ARM_L": [("arm_flex_l", sagittal, -ax_y, 2, 1, sag_idx), ("arm_add_l", frontal, -ax_y, 0, 3, front_idx), ("arm_rot_l", transversal, -ax_z, 0, 2, trans_idx)],
    "FOREARM_R": [("elbow_flex_r", sagittal, -ax_y, 2, 1, sag_idx), ("pro_sup_r", transversal, -ax_x, 0, 2, trans_idx)],
    "FOREARM_L": [("elbow_flex_l", sagittal, -ax_y, 2, 1, sag_idx), ("pro_sup_l", transversal, -ax_x, 0, 2, trans_idx)],
}

def measure_body(animation):
    jframes = {}
    for jname, _ in animation[0].items():
        jframes[jname] = len(animation) * [None]
    for frame_i in range(len(animation)):
        joints = animation[frame_i]
        for jname, transformation in joints.items():
            p = transformation[:3]
            jframes[jname][frame_i] = p.tolist()
    for jname, v in jframes.items():
        jpos = np.array(v, dtype=np.float32)
        mpos = np.median(jpos, axis=0)
        jframes[jname] = mpos
    blen = {}
    for jname, cpos in jframes.items():
        if jname not in child_to_parent:
            continue
        pname = child_to_parent[jname]
        ppos = jframes[pname]
        jlen = np.linalg.norm(cpos - ppos)
        bid = pname + "/" + jname
        blen[bid] = jlen
    # measure the size of a person
    p_toel = jframes["TOE_L"]
    p_toer = jframes["TOE_R"]

    toe_mid = p_toel + 0.5 * (p_toer - p_toel)

    p_head = jframes["HEAD"]
    height = np.linalg.norm(p_head - toe_mid)
    return blen, jframes, height