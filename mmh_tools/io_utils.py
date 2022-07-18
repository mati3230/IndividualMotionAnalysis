import xml.etree.ElementTree as ET
import numpy as np
from tqdm import tqdm
import h5py


def get_animation(frames):
    """Extracts the joint rotations of Skeleton_0 and the corresponding frame rotations. 
    
    Parameters
    ----------
    frames : xml.etree.ElementTree
        The .xml-tree of the .mmh-file.
        
    Return
    ------
    list, list
        Joint rotations for each frame (which are stored in dictionaries, one dictionary
        per frame for each joint) and the frame durations as list of floats. 
    """
    frame_list = []
    frame_durations = []
    for frame in frames: # for each frame
        for elem in frame:
            if elem.tag == "Skeleton_0":
                # init a dictionary for each frame (key: joint_name, value: transformation)
                joints = {}
                for joint in elem:
                    #print(joint.tag)
                    # init the transformation values
                    px = py = pz = 0
                    rx = ry = rz = 0
                    rw = 1
                    for t in joint:
                        if t.tag == "position":
                            px = float(t[0].text)
                            py = float(t[1].text)
                            pz = float(t[2].text)
                            # print(x, y, z)
                        elif t.tag == "rotation":
                            
                            rx = float(t[0].text)
                            ry = float(t[1].text)
                            rz = float(t[2].text)
                            rw = float(t[3].text)
                            #print(x, y, z, w)
                    """
                    extract the joint name - each .mmh joint starts with Joint_<joint_name>.
                    Therefore, we filter the "Joint_"-part from the .xml-tag.
                    """
                    joint_comps = joint.tag.split("_")[1:]
                    name = joint_comps[0]
                    side = ""
                    if len(joint_comps) > 1:
                        side = "_" + joint_comps[1]
                    joint_name = name + side
                    # add the dictionary entry
                    joints[joint_name] = np.array([px, py, pz, rx, ry, rz, rw], dtype=np.float32)
                    
                frame_list.append(joints)
            elif elem.tag == "frametime":
                frame_durations.append(float(elem.text))
    return frame_list, np.array(frame_durations, dtype=np.float32)


def load_mmh(filename):
    """Loads the mmh-file. 
    
    Parameters
    ----------
    filename : str
        Path to the mmh-File.

    Return
    ------
    list, list
        Joint rotations for each frame (which are stored in dictionaries, one dictionary
        per frame for each joint) and the frame durations as list of floats. 
    """
    xml_data = open(filename, "r").read()
    frames = ET.XML(xml_data)
    animation, durations = get_animation(frames=frames)
    return animation, durations


def save_mot_euler(filename, animation, n_times, dof_names=["TX", "TY", "TZ", "RX", "RY", "RZ"], ignore=[], root_j="TOE_L", name="mmh"):
    # write the normalized times and the joint angles
    columnNames = "time "
    # revert the last blank

    nColumns = 1
    for dof_name, dof in animation[0].items():
        if dof_name in ignore:
            continue
        for feat in dof_names:
            columnNames += dof_name + "_" + feat + " "
            nColumns += 1
    columnNames = columnNames[:-1]

    name = name
    version = 1
    nRows = len(n_times)
    inDegrees = "yes"
    endheader = "endheader"

    # write the header
    header = name + "\n"
    header += "version=" + str(version) + "\n"
    header += "nRows=" + str(nRows) + "\n"
    header += "nColumns=" + str(nColumns) + "\n"
    header += "inDegrees=" + inDegrees + "\n"
    header += endheader

    data = ""
    for i in range(len(n_times)):
        #print(n_time, angle)
        n_time = n_times[i]
        data += "\n" + "{0:.8f}".format(n_time)
        for dof_name, dof in animation[i].items():
            if dof_name in ignore:
                continue
            for j in range(len(dof_names)):
                if j < 3:
                    val = 0
                    if dof_name == root_j:
                        val = dof[j]
                    data += " " + "{0:.8f}".format(val)
                    continue
                #if (dof_name == "ARM_L" or dof_name == "ARM_R") and j == 5:
                #    data += " " + "{0:.8f}".format(-dof[j]) 
                #    continue   
                data += " " + "{0:.8f}".format(dof[j])

    mot = header + "\n"
    mot += columnNames
    mot += data

    with open(filename, "w") as f:
        f.write(mot)


def save_mot(filename, joint_anim_cp, n_times):
    name = "mmh"
    version = 1
    nRows = len(n_times)
    nColumns = len(joint_anim_cp) + 1
    inDegrees = "yes"
    endheader = "endheader"

    # write the header
    header = name + "\n"
    header += "version=" + str(version) + "\n"
    header += "nRows=" + str(nRows) + "\n"
    header += "nColumns=" + str(nColumns) + "\n"
    header += "inDegrees=" + inDegrees + "\n"
    header += endheader

    # write the normalized times and the joint angles
    columnNames = "time "

    for dof_name, angles in joint_anim_cp.items():
        columnNames += dof_name + " "
    # revert the last blank
    columnNames = columnNames[:-1]

    data = ""
    for i in range(len(n_times)):
        #print(n_time, angle)
        n_time = n_times[i]
        data += "\n" + "{0:.8f}".format(n_time)
        for dof_name, angles in joint_anim_cp.items():
            data += " " + "{0:.8f}".format(angles[i])

    mot = header + "\n"
    mot += columnNames
    mot += data

    with open(filename, "w") as f:
        f.write(mot)


def save_h5(fname, durations, anim, n_times):
    d = anim[0]["HIPS"].shape[0]
    n = len(anim)
    #print(n, d)
    #print(list(anim[0].keys()))
    all_frames = {}
    for key in list(anim[0].keys()):
        all_frames[key] = np.zeros((n,d))
    for i in tqdm(range(len(anim)), desc="Save"):
        joints = anim[i]
        for k, v in joints.items():
            all_frames[k][i] = v
    hf = h5py.File("{0}.h5".format(fname), "w")
    for k, v in all_frames.items():
        hf.create_dataset(k, data=v)
    hf.create_dataset("n",data=np.array([n]))
    hf.create_dataset("n_times", data=n_times)
    hf.close()


def save_csv(fname, durations, anim, is_euler=False):
    header = "Frame_Duration"
    for k, v in anim[0].items():
        header += ", {0}_px, {1}_py, {2}_pz".format(k, k, k)
        if is_euler:
            header += ", {0}_ex, {1}_ey, {2}_ez".format(k, k, k)
        else:
            header += ", {0}_qx, {1}_qy, {2}_qz, {3}_qw".format(k, k, k, k)
    # list of lists
    rows = len(anim)*[None]
    for i in tqdm(range(len(anim)), desc="Save"):
        duration = durations[i]
        joints = anim[i]
        # first value in a row are the durations
        row = [duration]
        for k, v in joints.items():
            row.extend(v.tolist())
        rows[i] = row
    data = np.array(rows, dtype=np.float32)
    np.savetxt(fname=fname, X=data, header=header, delimiter=",")


def save_body_csv(fname, height, segment_lengths, avg_positions):
    header = "Height"
    data = [height]
    for k, v in segment_lengths.items():
        header += ", {0}".format(k)
        data.append(v)
    for k, v in avg_positions.items():
        header += ", {0}".format(k)
        data.extend(v.tolist())
    data = np.array(data, dtype=np.float32)
    data = data.reshape(data.shape[0], 1)
    data = data.transpose()
    np.savetxt(fname=fname, X=data, header=header, delimiter=",")