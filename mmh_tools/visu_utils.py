import open3d as o3d
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

from angle_utils import get_parent_p, get_parent_q, q_from_numpy, rotate
from skeleton_utils import ax_x, ax_y, ax_z, skeleton, skeleton_osim, osim_child_to_parent, child_to_parent


# x-Axis right, y-Axis: up, z-Axis: points away from me
def coordinate_system(x, y, z):
    """Generates a line set for the coordinate axis. 
        
    Return
    ------
    o3d.geometry.LineSet
        A line for each coordinate axis.
    """
    line_set = o3d.geometry.LineSet()
    points = np.array([[0, 0, 0], x.tolist(), y.tolist(), z.tolist()])
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    lines = np.array([[0, 1], [0, 2], [0, 3]]).astype(int)
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    return line_set


def dof_lines(out, out1=None, color = [1, 1, 0], def_color = [0, 1, 1]):
    line_points = []
    lines = []
    i=0
    colors = []

    if out1 is not None:
        for k, v in out1.items():
            root_pos = v[0]
            x_pos = root_pos + v[1]
            y_pos = root_pos + v[2]
            z_pos = root_pos + v[3]
            line_points.append(root_pos.tolist())
            line_points.append(x_pos.tolist())
            line_points.append(y_pos.tolist())
            line_points.append(z_pos.tolist())
            lines.append([i, i+1])
            lines.append([i, i+2])
            lines.append([i, i+3])
            colors.append([1, 0, 0])
            colors.append([0, 1, 0])
            colors.append([0, 0, 1])
            i += 4
    """
    for k, v in out.items():
        dof_name = v[0]
        pos = v[1]
        vec = pos + v[2]
        default = pos + v[3]
        line_points.append(pos.tolist())
        line_points.append(vec.tolist())
        line_points.append(default.tolist())
        lines.append([i, i+1])
        colors.append(color)
        lines.append([i, i+2])
        colors.append(def_color)
        if out1 is not None:
            vx, vy, vz = out1[k]
            px = vx + pos
            py = vy + pos
            pz = vz + pos
            line_points.append(px.tolist())
            line_points.append(py.tolist())
            line_points.append(pz.tolist())
            lines.append([i, i+3])
            colors.append([1, 0, 0])
            lines.append([i, i+4])
            colors.append([0, 1, 0])
            lines.append([i, i+5])
            colors.append([0, 0, 1])
            i+=6
        else:
            i+=3
    """
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(line_points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def skeleton_lines(joint_points, skeleton, color=[0, 0, 0]):
    """Returns a line set of the skeleton. 
    
    Parameters
    ----------
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    skeleton : dict
        Skeleton dictionary that is defined above.
    color : np.ndarray
        Skeleton will be drawn in this RGB color.
        
    Return
    ------
    o3d.geometry.LineSet
        Line set of the skeleton. 
    """
    line_points = [] # points of the skeleton
    joint_i = 0
    joint_to_idx = {} # stores the list index (of the list 'line_points') for each joint
    
    for j_name, j_pos in joint_points.items():
        # print(j_name, j_pos)
        line_points.append(j_pos.tolist())
        joint_to_idx[j_name] = joint_i
        joint_i += 1

    # create a line for each parent to child joint connection
    lines = [] # a line consist of the indices of two points
    for j_name, child_dict in skeleton.items():
        parent_i = joint_to_idx[j_name]
        for end_joint_name, _ in child_dict.items():
            child_i = joint_to_idx[end_joint_name]
            lines.append([parent_i, child_i])

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(line_points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    # assign the line colors
    colors = [color for i in range(len(lines))]
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def text_3d(text, pos):
    img = Image.new('RGB', (64, 64), color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, fill=(0, 0, 0))
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 128
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T

    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
    pcd.points = o3d.utility.Vector3dVector(indices / 100.0 + pos)
    return pcd


def update_vis(vis, sls, cls, frame_nr, show_frame_nr=False, out_line_set=None):
    """Draw a skeleton frame by adding new geometry to 
    the visualizer for a short moment.
    
    Parameters
    ----------
    vis : o3d.visualization.Visualizer
        Stores a position for each joint.
    sls : o3d.geometry.LineSet
        Skeleton line set.
    cls : o3d.geometry.LineSet
        Line set of the coordinate system.
    """
    vis.add_geometry(sls)
    vis.add_geometry(cls)
    if out_line_set is not None:
        vis.add_geometry(out_line_set)
    if show_frame_nr:
        im = text_3d(text=str(frame_nr), pos=np.array([0, 1, 0]))
        vis.add_geometry(im)
    vis.poll_events()
    vis.update_renderer()
    #vis.capture_screen_image(filename="../frames_pg/"+str(frame_nr) + ".jpg", do_render=False)
    vis.clear_geometries()


def render_skeleton_animation(
        animation,
        skeleton_render_func,
        render_frame=-1,
        show_frame_nr=False,
        use_mmh_skeleton=False):
    """Renders a skeleton animation with the Open3D visualizer.
    
    Parameters
    ----------
    joints : dict[str, np.ndarray]
        An animation frame, i.e. a transformation for each joint.
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    skeleton_render_func : func
        A function that sets the joint positions (called 'joint_points').
    """
    # create a visualizer
    if render_frame == -1:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="MMH Animation", width=640, height=640)
    co_line_set = coordinate_system(x=ax_x, y=ax_y, z=ax_z)
    skl = skeleton_osim
    j_root = "TOE_L"
    ctop_dict = osim_child_to_parent
    if use_mmh_skeleton:
        skl = skeleton
        j_root = "HIPS"
        ctop_dict = child_to_parent
    #j_root = "HIPS"
    joint_points = {}
    frame = 0
    for joints in animation:
        if render_frame != -1 and frame != render_frame:
            frame += 1
            continue
        # store the root joint position
        root_position = joints[j_root][:3]
        #joint_points[j_root] = np.zeros((3,))
        joint_points[j_root] = root_position
        #child_joints = skeleton[j_root]
        # render the remaining skeleton
        out={}
        out1 = {}
        skeleton_render_func(skl=skl,
                             joints=joints,
                             joint_points=joint_points,
                             parent_joint=j_root,
                             ctop_dict=ctop_dict,
                             root_j=j_root,
                             info=None,
                             out=out,
                             out1=out1)
        # get the o3d.geometry.LineSet
        skeleton_line_set = skeleton_lines(joint_points=joint_points, skeleton=skl)
        if len(out1) > 0:
            out_line_set = dof_lines(out=out, out1=out1)
        # display the current frame on the visualizer
        if render_frame == -1:
            if len(out1) > 0:
                update_vis(vis=vis, sls=skeleton_line_set, cls=co_line_set, frame_nr=frame, show_frame_nr=show_frame_nr, out_line_set=out_line_set)
            else:
                update_vis(vis=vis, sls=skeleton_line_set, cls=co_line_set, frame_nr=frame, show_frame_nr=show_frame_nr)
        else:
            im = text_3d(text=str(frame), pos=np.array([0, 1, 0]))
            if len(out1) > 0:
                o3d.visualization.draw_geometries([skeleton_line_set, co_line_set, im, out_line_set])
            else:
                o3d.visualization.draw_geometries([skeleton_line_set, co_line_set, im])
            #o3d.visualization.draw_geometries([skeleton_line_set, im, out_line_set])
        frame += 1
    if render_frame == -1:
        vis.destroy_window()


def apply_pos_skeleton(skl, joints, joint_points, parent_joint, ctop_dict , root_j, info=None, out=None, out1=None):
    """Applies the joint positions of each frame. The function stores for each
    joint a joint position in 'joint_points'. The skeleton is iterated is a DFS.

    Parameters
    ----------
    skl : dict
        Skeleton dictionary that is defined above.
    joints : dict[str, np.ndarray]
        An animation frame, i.e. a transformation for each joint.
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    parent_joint : str
        Name of the parent joint.
    """
    # get the child joints
    child_joints = skl[parent_joint]
    # for every child joint
    for end_joint_name, joint_v in child_joints.items():
        # what is the transformation of the specific joint
        transformation = joints[end_joint_name]
        # store the joint position
        joint_points[end_joint_name] = transformation[:3]
        # repeat this function for the child joints of this child joint.
        if end_joint_name in skl:
            apply_pos_skeleton(skl=skl,  
                                 joints=joints, 
                                 joint_points=joint_points,
                                 parent_joint=end_joint_name,
                                 ctop_dict=ctop_dict,
                                 root_j=root_j,
                                 info=info,
                                 out=out,
                                 out1=out1)


def apply_local_pos_skeleton(skl, joints, joint_points, parent_joint, ctop_dict, root_j, info=None, out=None, out1=None):
    """Applies the joint positions of each frame. The function stores for each
    joint a joint position in 'joint_points'. The skeleton is iterated is a DFS.

    Parameters
    ----------
    skeleton : dict
        Skeleton dictionary that is defined above.
    joints : dict[str, np.ndarray]
        An animation frame, i.e. a transformation for each joint.
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    parent_joint : str
        Name of the parent joint.
    """

    if parent_joint == root_j:
        info = {}
        info[parent_joint] = joints[parent_joint][:3]
    else:
        _, pp_name = get_parent_p(joints=joints, name=parent_joint, ctop_dict=ctop_dict)
        p_local = joints[parent_joint][:3]
        info[parent_joint] = info[pp_name] + p_local

    # get the child joints
    child_joints = skl[parent_joint]
    # for every child joint
    for end_joint_name, joint_v in child_joints.items():
        # what is the transformation of the specific joint
        transformation = joints[end_joint_name]
        p = transformation[:3]
        # store the joint position
        joint_points[end_joint_name] = info[parent_joint] + p
        # repeat this function for the child joints of this child joint.
        if end_joint_name in skl:
            apply_local_pos_skeleton(skl=skl,  
                                 joints=joints, 
                                 joint_points=joint_points,
                                 parent_joint=end_joint_name,
                                 ctop_dict=ctop_dict,
                                 root_j=root_j,
                                 info=info,
                                 out=out,
                                 out1=out1)


def apply_rot_skeleton(skl, joints, joint_points, parent_joint, ctop_dict, root_j, info=None, out=None, out1=None):
    """Applies the joint rotations of each frame to the joint vectors. The function stores for each
    joint a joint position in 'joint_points'. The skeleton is iterated is a DFS.
    
    Parameters
    ----------
    skl : dict
        Skeleton dictionary that is defined above.
    joints : dict[str, np.ndarray]
        An animation frame, i.e. a transformation for each joint.
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    parent_joint : str
        Name of the parent joint.
    """
    # get the child joints
    child_joints = skl[parent_joint]
    # get the transformation of the parent joint
    parent_trans = joints[parent_joint]
    # get the latest position of the parent joint
    parent_pos = joint_points[parent_joint]
    # get the rotation of the animation frame of that joint
    parent_rot = parent_trans[3:]

    #rp = Quaternion(parent_rot[3], parent_rot[0], parent_rot[1], parent_rot[2])
    r = Quaternion(parent_rot[3], parent_rot[0], parent_rot[1], parent_rot[2])
    out1[parent_joint] = (parent_pos, r.rotate(0.1*ax_x), r.rotate(0.1*ax_y), r.rotate(0.1*ax_z))
    #print(parent_joint)

    # for every child joint
    for end_joint_name, joint_v in child_joints.items():
        # create a scipy quaternion rotation
        #r = R.from_quat(parent_rot)
    
        #q_p = q_from_numpy(parent_rot)

        #q_c = Quaternion(axis=[0, 1, 0], angle=np.pi/2)
        #if parent_joint == "HIPS":
        # apply the rotation to the child joint vector in the skeleton model (global rotations are assumed) 
        #joint_v_ = r.apply(joint_v)
        joint_v_ =  r.rotate(joint_v)
        # update the child joint position
        joint_v__ = joint_v_ + parent_pos
        # store the joint position
        #joint_points[end_joint_name] = joint_v__
        joint_points[end_joint_name] = joints[end_joint_name][:3]
        
        # repeat this function for the child joints of this child joint.
        if end_joint_name in skl:
            apply_rot_skeleton(skl=skl,  
                                 joints=joints, 
                                 joint_points=joint_points,
                                 parent_joint=end_joint_name,
                                 ctop_dict=ctop_dict,
                                 root_j=root_j,
                                 info=info,
                                 out=out,
                                 out1=out1)
        elif end_joint_name not in out1:
            c_rot = joints[end_joint_name][3:]
            cr = Quaternion(c_rot[3], c_rot[0], c_rot[1], c_rot[2])
            out1[end_joint_name] = (joint_points[end_joint_name], cr.rotate(0.1*ax_x), cr.rotate(0.1*ax_y), cr.rotate(0.1*ax_z))


def apply_local_rot_skeleton(skl, joints, joint_points, parent_joint, ctop_dict, root_j, info=None, out=None, out1=None):
    """Applies the joint rotations of each frame to the joint vectors. The function stores for each
    joint a joint position in 'joint_points'. The skeleton is iterated is a DFS.
    
    Parameters
    ----------
    skl : dict
        Skeleton dictionary that is defined above.
    joints : dict[str, np.ndarray]
        An animation frame, i.e. a transformation for each joint.
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    parent_joint : str
        Name of the parent joint.
    """
    # get the child joints
    child_joints = skl[parent_joint]
    # get the transformation of the parent joint
    parent_trans = joints[parent_joint]
    # get the latest position of the parent joint
    parent_pos = joint_points[parent_joint]
    # get the rotation of the animation frame of that joint
    parent_rot = parent_trans[3:]
    
    q_p_l = q_from_numpy(parent_rot)
    
    if parent_joint == root_j:
        info = {}
        info[parent_joint] = q_p_l
    else:
        q_pp_g, pp_name = get_parent_q(joints=joints, name=parent_joint, ctop_dict=ctop_dict)
        info[parent_joint] = info[pp_name] * q_p_l
        
    # for every child joint
    q_c_g = info[parent_joint]
    for end_joint_name, joint_v in child_joints.items():

        joint_v_ = rotate(q_c_g, joint_v)
        joint_v__ = joint_v_ + parent_pos
        joint_points[end_joint_name] = joint_v__
        
        # repeat this function for the child joints of this child joint.
        if end_joint_name in skl:
            apply_local_rot_skeleton(skl=skl,  
                                 joints=joints, 
                                 joint_points=joint_points,
                                 parent_joint=end_joint_name,
                                 ctop_dict=ctop_dict,
                                 root_j=root_j,
                                 info=info,
                                 out=out,
                                 out1=out1)


def apply_euler_skeleton(skl, joints, joint_points, parent_joint, ctop_dict, root_j, info=None, out=None, out1=None):
    """Applies the joint rotations of each frame to the joint vectors. The function stores for each
    joint a joint position in 'joint_points'. The skeleton is iterated is a DFS.
    
    Parameters
    ----------
    skl : dict
        Skeleton dictionary that is defined above.
    joints : dict[str, np.ndarray]
        An animation frame, i.e. a transformation for each joint.
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    parent_joint : str
        Name of the parent joint.
    """
    # get the child joints
    child_joints = skl[parent_joint]
    # get the transformation of the parent joint
    parent_trans = joints[parent_joint]
    # get the latest position of the parent joint
    parent_pos = joint_points[parent_joint]
    # get the rotation of the animation frame of that joint
    parent_rot = parent_trans[3:]
    
    r_p_l = R.from_euler(angles=parent_rot, seq="xyz", degrees=True)
    M_p_l = r_p_l.as_matrix()

    if parent_joint == root_j:
        info = {}
        info[parent_joint] = M_p_l
        #print(parent_joint, parent_pos)
    else:
        pp_name = ctop_dict[parent_joint]
        info[parent_joint] = np.matmul(info[pp_name], M_p_l)
        
    # for every child joint
    M_c_g = info[parent_joint]
    print(parent_joint, M_c_g)

    """
    out[parent_joint + str(j)] = (dof_name, parent_pos, rotated_vec, default_vec)
    vx = rotate(quat=q_local, vec=ax_x)
    vy = rotate(quat=q_local, vec=ax_y)
    vz = rotate(quat=q_local, vec=ax_z)
    #vx = rotate(quat=q_p_, vec=ax_x)
    #vy = rotate(quat=q_p_, vec=ax_y)
    #vz = rotate(quat=q_p_, vec=ax_z)
    out1[parent_joint + str(j)] = (vx, vy, vz)
    """
    out1[parent_joint] = (parent_pos, np.matmul(M_c_g, 0.1*ax_x), np.matmul(M_c_g, 0.1*ax_y), np.matmul(M_c_g, 0.1*ax_z))

    for end_joint_name, joint_v in child_joints.items():
        #joint_v_ = np.matmul(M_c_g, joint_v)
        #joint_v__ = joint_v_ + parent_pos
        #joint_points[end_joint_name] = joint_v__
        joint_points[end_joint_name] = joint_points[parent_joint] + joints[end_joint_name][:3]
        #print(end_joint_name, joint_points[end_joint_name])
        
        # repeat this function for the child joints of this child joint.
        if end_joint_name in skl:
            apply_euler_skeleton(skl=skl,  
                                 joints=joints, 
                                 joint_points=joint_points,
                                 parent_joint=end_joint_name,
                                 ctop_dict=ctop_dict,
                                 root_j=root_j,
                                 info=info,
                                 out=out,
                                 out1=out1)
        elif end_joint_name not in out1:
            c_rot = joints[end_joint_name][3:]
            r_p_l = R.from_euler(angles=c_rot, seq="xyz", degrees=True)
            M_p_l = r_p_l.as_matrix()
            #pp_name = ctop_dict[parent_joint]
            M = np.matmul(info[parent_joint], M_p_l)
            out1[end_joint_name] = (joint_points[end_joint_name], np.matmul(M, 0.1*ax_x), np.matmul(M, 0.1*ax_y), np.matmul(M, 0.1*ax_z))


def apply_euler_skeleton_global(skl, joints, joint_points, parent_joint, ctop_dict, root_j, info=None, out=None, out1=None):
    """Applies the joint rotations of each frame to the joint vectors. The function stores for each
    joint a joint position in 'joint_points'. The skeleton is iterated is a DFS.
    
    Parameters
    ----------
    skl : dict
        Skeleton dictionary that is defined above.
    joints : dict[str, np.ndarray]
        An animation frame, i.e. a transformation for each joint.
    joint_points : dict[str, np.ndarray]
        Stores a position for each joint.
    parent_joint : str
        Name of the parent joint.
    """
    # get the child joints
    child_joints = skl[parent_joint]
    # get the transformation of the parent joint
    parent_trans = joints[parent_joint]
    # get the latest position of the parent joint
    parent_pos = joint_points[parent_joint]
    # get the rotation of the animation frame of that joint
    parent_rot = parent_trans[3:]
    
    r_p_g = R.from_euler(angles=parent_rot, seq="xyz", degrees=True)
    M_c_g = r_p_g.as_matrix()

    #if parent_joint == "HIPS":
    #    print(M_c_g)

    """
    out[parent_joint + str(j)] = (dof_name, parent_pos, rotated_vec, default_vec)
    vx = rotate(quat=q_local, vec=ax_x)
    vy = rotate(quat=q_local, vec=ax_y)
    vz = rotate(quat=q_local, vec=ax_z)
    #vx = rotate(quat=q_p_, vec=ax_x)
    #vy = rotate(quat=q_p_, vec=ax_y)
    #vz = rotate(quat=q_p_, vec=ax_z)
    out1[parent_joint + str(j)] = (vx, vy, vz)
    """
    out1[parent_joint] = (parent_pos, np.matmul(M_c_g, 0.1*ax_x), np.matmul(M_c_g, 0.1*ax_y), np.matmul(M_c_g, 0.1*ax_z))

    for end_joint_name, joint_v in child_joints.items():
        #joint_v_ = np.matmul(M_c_g, joint_v)
        #joint_v__ = joint_v_ + parent_pos
        #joint_points[end_joint_name] = joint_v__
        joint_points[end_joint_name] = joints[end_joint_name][:3]

        
        # repeat this function for the child joints of this child joint.
        if end_joint_name in skl:
            apply_euler_skeleton_global(skl=skl,  
                                 joints=joints, 
                                 joint_points=joint_points,
                                 parent_joint=end_joint_name,
                                 ctop_dict=ctop_dict,
                                 root_j=root_j,
                                 info=info,
                                 out=out,
                                 out1=out1)
        elif end_joint_name not in out1:
            c_rot = joints[end_joint_name][3:]
            r_p_l = R.from_euler(angles=c_rot, seq="xyz", degrees=True)
            M = r_p_l.as_matrix()
            #pp_name = ctop_dict[parent_joint]
            #M = np.matmul(info[parent_joint], M_p_l)
            out1[end_joint_name] = (joint_points[end_joint_name], np.matmul(M, 0.1*ax_x), np.matmul(M, 0.1*ax_y), np.matmul(M, 0.1*ax_z))