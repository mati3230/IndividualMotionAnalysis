import numpy as np
from pyquaternion import Quaternion

from skeleton_utils import skeleton,\
                            sag_idx, front_idx, trans_idx,\
                            flex_vec, add_vec


def q_identity():
    """ Returns the identity quaternion. 
    
    Return
    ------
    np.ndarray
        The identity quaternion.
    """
    #return R.from_quat(np.array([0, 0, 0, 1]))
    return Quaternion(1, 0, 0, 0)


def q_from_numpy(q):
    return Quaternion(q[3], q[0], q[1], q[2])


def q_from_vec(v):
    return Quaternion(0, v[0], v[1], v[2])


def rotate(quat, vec):
    qv = q_from_vec(vec)
    q = quat * qv * quat.inverse
    return np.array([q[1], q[2], q[3]])


def get_q(joints, name):
    r_g = joints[name][3:]
    q = q_from_numpy(r_g)
    return q


def get_p(joints, name):
    p = joints[name][:3]
    return p


def get_parent_q(joints, name, ctop_dict):
    p_name = ctop_dict[name]
    return get_q(joints=joints, name=p_name), p_name


def get_parent_p(joints, name, ctop_dict):
    p_name = ctop_dict[name]
    return get_p(joints=joints, name=p_name), p_name


def angle_vector(a, b):    
    angle = dot(a=a, b=b)[0]
    angle = np.arccos(angle)
    # convert the angle to degrees
    angle = 180 * angle / np.pi
    return angle


def get_angle(joint_name, dof_name, q_local, q_idx, default_vec, plane_n, axis_idx):
    q_l = Quaternion(q_local)
    q_local_ = Quaternion(q_l)
    rotated_vec = rotate(quat=q_local_, vec=default_vec)
    rotated_vec_proj = project(u=rotated_vec, n=plane_n)
    angle = angle_vector(a=default_vec, b=rotated_vec_proj)
    if rotated_vec_proj[axis_idx] > 0:
        angle *= -1
    return angle, rotated_vec, rotated_vec_proj
    

def get_flex_add(joints, joint_name, idx):
    child_name = list(skeleton[joint_name].keys())[0]
    p_pos = joints[joint_name][:3]
    j_pos = joints[child_name][:3]
    joint_v = j_pos - p_pos
    joint_v /= np.linalg.norm(joint_v)
    if idx == sag_idx:
        return angle_vector(a=flex_vec, b=joint_v)
    elif idx == front_idx:
        angle = angle_vector(a=add_vec, b=joint_v) - 90 
        if joint_v[0] < 0:
            angle *= -1
        return angle


def get_in_between(joints, joint_name, ctop_dict):
    child_name = list(skeleton[joint_name].keys())[0]
    parent_name = ctop_dict[joint_name]
    pos = joints[joint_name][:3]
    c_pos = joints[child_name][:3]
    p_pos = joints[parent_name][:3]
    v1 = p_pos - pos
    v2 = c_pos - pos
    return 180 - angle_vector(a=v1, b=v2)


def project(u, n): 
    """ Function to project a vector u onto a plane with the normal n. 
    
    Parameters
    ----------
    u : np.ndarray
        Vector that should be projected onto a plane.
    n : np.ndarray
        The normal of the plane.
        
    Return
    ------
    np.ndarray
        The projected vector.
    """
    n_norm = np.sqrt(sum(n**2))
    proj_of_u_on_n = (unnormalized_dot(u, n)/n_norm**2)*n
    proj_of_u_on_n = u - proj_of_u_on_n
    return proj_of_u_on_n


def unnormalized_dot(a, b):
    dot = a*b
    dot = np.sum(dot, axis=-1)
    return dot
    

def dot(a, b, eps=1e-6):
    """ Function to calculate a dot product between two vectors. 
    
    Parameters
    ----------
    a : np.ndarray
        Vector - should have the same size as b.
    b : np.ndarray
        Vector.
    eps : float
        Small value to deny a devition by zero.
        
    Return
    ------
    float, float, float
        The dot product with the two corresponding vector norms.
    """
    dot = a*b
    dot = np.sum(dot, axis=-1)
    a_n = np.linalg.norm(a, axis=-1)
    b_n = np.linalg.norm(b, axis=-1)
    if a_n == 0 or b_n == 0:
        dot /= ((a_n * b_n) + eps)
    else:
        dot /= (a_n * b_n)
    return dot, a_n, b_n


def negate(dof_name, joint_anim):
    """ Negates certain angles. 
    
    Parameters
    ----------
    dof_name : str
        Name of the degree of freedom that should be negated.
    joint_anim : dict[str, np.ndarray]
        Joint angles for each degree of freedom.
        
    Return
    ------
    dict[str, np.ndarray]
        Joint angles for each degree of freedom.
    """
    angle_arr = -joint_anim[dof_name]
    joint_anim[dof_name] = angle_arr
    return joint_anim


def add_offset(dof_name, joint_anim, offset=0):
    """ Adds an offset to certain angles. 
    
    Parameters
    ----------
    dof_name : str
        Name of the degree of freedom.
    joint_anim : dict[str, np.ndarray]
        Joint angles for each degree of freedom.
    offset : float
        Angle offset that will be added to each angle.
        
    Return
    ------
    dict[str, np.ndarray]
        Joint angles for each degree of freedom.
    """
    angle_arr = joint_anim[dof_name]
    joint_anim[dof_name] = angle_arr + offset
    return joint_anim


def delete(dof_name, joint_anim):
    """ TODO 
    
    Parameters
    ----------
    dof_name : str
        Name of the degree of freedom.
    joint_anim : dict[str, np.ndarray]
        Joint angles for each degree of freedom.
        
    Return
    ------
    dict[str, np.ndarray]
        Joint angles for each degree of freedom.
    """
    angle_arr = joint_anim[dof_name]
    joint_anim[dof_name] = np.zeros((angle_arr.shape[0], ))
    return joint_anim


def switch(dn1, dn2, joint_anim):
    """ Switch the angles of two degrees of freedom. 
    
    Parameters
    ----------
    dn1 : str
        Name of the degree of freedom.
    dn2 : str
        Name of the degree of freedom.
    joint_anim : dict[str, np.ndarray]
        Joint angles for each degree of freedom.
        
    Return
    ------
    dict[str, np.ndarray]
        Joint angles for each degree of freedom.
    """
    a1 = np.array(joint_anim[dn1], copy=True)
    a2 = np.array(joint_anim[dn2], copy=True)
    joint_anim[dn1] = a2
    joint_anim[dn2] = a1
    return joint_anim