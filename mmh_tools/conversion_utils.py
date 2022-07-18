import numpy as np
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
import math
import sys

from skeleton_utils import skeleton_angles,\
                            child_to_parent,\
                            osim_child_to_parent,\
                            skeleton,\
                            ax_x, ax_y, ax_z,\
                            sag_idx, front_idx, trans_idx
from angle_utils import q_from_numpy,\
                        get_parent_q,\
                        get_parent_p,\
                        rotate,\
                        angle_vector,\
                        get_angle,\
                        get_in_between,\
                        get_flex_add



def rot_mat2euler(R):
    """
    //------------------------------------------------------------------------------
    // Calculate angles ONLY for a three-angle, three-axes, body-fixed, ijk rotation 
    // sequence where i != j and j != k.
    //------------------------------------------------------------------------------
    template <class P> Vec<3,P>
    Rotation_<P>::convertThreeAxesBodyFixedRotationToThreeAngles
      ( const CoordinateAxis& axis1, const CoordinateAxis& axis2, 
        const CoordinateAxis& axis3 ) const 
    {
        // Ensure this method has proper arguments.
        assert( axis1.areAllDifferentAxes(axis2,axis3) );

        const int i = int(axis1);
        const int j = int(axis2);
        const int k = int(axis3);

        // Need to know if using a forward or reverse cyclical.
        RealP plusMinus = 1.0,  minusPlus = -1.0;
        if( axis1.isReverseCyclical(axis2) ) { plusMinus = -1.0,  minusPlus = 1.0; }

        // Shortcut to the elements of the rotation matrix.
        const Mat33P& R = asMat33();

        // Calculate theta2 using lots of information in the rotation matrix.
        RealP Rsum   =  std::sqrt((  square(R[i][i]) + square(R[i][j]) 
                                   + square(R[j][k]) + square(R[k][k])) / 2);
        // Rsum = abs(cos(theta2)) is inherently positive.
        RealP theta2 =  std::atan2( plusMinus*R[i][k], Rsum ); 
        RealP theta1, theta3;

        // There is a "singularity" when cos(theta2) == 0
        if( Rsum > 4*Eps ) {
            theta1 =  std::atan2( minusPlus*R[j][k], R[k][k] );
            theta3 =  std::atan2( minusPlus*R[i][j], R[i][i] );
        }
        else if( plusMinus*R[i][k] > 0 ) {
            const RealP spos = R[j][i] + plusMinus*R[k][j];  // 2*sin(theta1 + plusMinus*theta3)
            const RealP cpos = R[j][j] + minusPlus*R[k][i];  // 2*cos(theta1 + plusMinus*theta3)
            const RealP theta1PlusMinusTheta3 = std::atan2( spos, cpos );
            theta1 = theta1PlusMinusTheta3;  // Arbitrary split
            theta3 = 0;                      // Arbitrary split
        }
        else {
            const RealP sneg = plusMinus*(R[k][j] + minusPlus*R[j][i]);  // 2*sin(theta1 + minusPlus*theta3)
            const RealP cneg = R[j][j] + plusMinus*R[k][i];              // 2*cos(theta1 + minusPlus*theta3)
            const RealP theta1MinusPlusTheta3 = std::atan2( sneg, cneg );
            theta1 = theta1MinusPlusTheta3;  // Arbitrary split
            theta3 = 0;                      // Arbitrary split
        }

        // Return values have the following ranges:
        // -pi   <=  theta1  <=  +pi
        // -pi/2 <=  theta2  <=  +pi/2   (Rsum is inherently positive)
        // -pi   <=  theta3  <=  +pi
        return Vec3P( theta1, theta2, theta3 );
    }
    """
    Eps = sys.float_info.epsilon
    i = 0
    j = 1
    k = 2
    
    plusMinus = 1.0
    minusPlus = -1.0
    # previous axis of xAxis is zAxis
    #if( xAxis.isPreviousAxis(yAxis) ) { plusMinus = -1.0,  minusPlus = 1.0; }
    Rsum = math.sqrt((R[i, i]**2 + R[i, j]**2 + R[j, k]**2 + R[k, k]**2) / 2)
    # Rsum = abs(cos(theta2)) is inherently positive.
    theta2 =  math.atan2( plusMinus*R[i, k], Rsum ) 
    
    if Rsum > 4*Eps:
        theta1 =  math.atan2( minusPlus*R[j, k], R[k, k] )
        theta3 =  math.atan2( minusPlus*R[i, j], R[i, i] )
    elif plusMinus*R[i, k] > 0:
        spos = R[j, i] + plusMinus*R[k, j]  # 2*sin(theta1 + plusMinus*theta3)
        cpos = R[j, j] + minusPlus*R[k, i]  # 2*cos(theta1 + plusMinus*theta3)
        theta1PlusMinusTheta3 = math.atan2( spos, cpos )
        theta1 = theta1PlusMinusTheta3  # Arbitrary split
        theta3 = 0                      # Arbitrary split
    else:
        sneg = plusMinus*(R[k, j] + minusPlus*R[j, i])  # 2*sin(theta1 + minusPlus*theta3)
        cneg = R[j, j] + plusMinus*R[k, i]              # 2*cos(theta1 + minusPlus*theta3)
        theta1MinusPlusTheta3 = math.atan2( sneg, cneg )
        theta1 = theta1MinusPlusTheta3  # Arbitrary split
        theta3 = 0                      # Arbitrary split
    # Return values have the following ranges:
    # -pi   <=  theta1  <=  +pi
    # -pi/2 <=  theta2  <=  +pi/2   (Rsum is inherently positive)
    # -pi   <=  theta3  <=  +pi
    return np.array([theta1, theta2, theta3])
    

def q2rot_mat(q):
    """
    template <class P> Rotation_<P>& Rotation_<P>::setRotationFromQuaternion( const Quaternion_<P>& q )  {
    const RealP q00=q[0]*q[0], q11=q[1]*q[1], q22=q[2]*q[2], q33=q[3]*q[3];
    const RealP q01=q[0]*q[1], q02=q[0]*q[2], q03=q[0]*q[3];
    const RealP q12=q[1]*q[2], q13=q[1]*q[3], q23=q[2]*q[3];
    const RealP q00mq11 = q00-q11, q22mq33 = q22-q33;

    Mat33P::operator=(Mat33P(q00+q11-q22-q33,  2*(q12-q03)  ,  2*(q13+q02),
                               2*(q12+q03)  ,q00mq11+q22mq33,  2*(q23-q01),
                               2*(q13-q02)  ,  2*(q23+q01)  ,q00mq11-q22mq33));
    return *this;
    }
    """
    # q = [w, x, y, z]
    q00=q[0]*q[0]
    q11=q[1]*q[1]
    q22=q[2]*q[2]
    q33=q[3]*q[3]
    
    q01=q[0]*q[1]
    q02=q[0]*q[2]
    q03=q[0]*q[3]

    q12=q[1]*q[2]
    q13=q[1]*q[3]
    q23=q[2]*q[3]
    
    q00mq11 = q00-q11
    q22mq33 = q22-q33

    rot_mat = np.array([
        [q00+q11-q22-q33,  2*(q12-q03)  ,  2*(q13+q02)],
        [2*(q12+q03)  ,q00mq11+q22mq33,  2*(q23-q01)],
        [2*(q13-q02)  ,  2*(q23+q01)  ,q00mq11-q22mq33]
    ], dtype=np.float32)
    return rot_mat


def body_center(joints):
    p_list = []
    for joint_name, transformation in joints.items():
        p_list.append(transformation[:3])
    center = np.mean(p_list, axis=0)
    return center


def convert_to_euler(animation, global_data=True, mx=-1, my=1, mz=-1):
    # container for the result of this function
    euler_anim = len(animation) * [None]
    rad2deg = (180/np.pi)

    for i in range(len(animation)):
        euler_anim[i] = {}
    for frame_i in tqdm(range(len(animation)), desc="Euler Conversion"):
        joints = animation[frame_i]
        #center = body_center(joints=joints)
        # read each joint and transformation per frame
        for joint_name, T in joints.items():
            # extract the joint position
            p = T[:3]

            R_mat = q2rot_mat(np.array([T[6], mx*T[3], my*T[4], mz*T[5]], dtype=np.float32))
            e = rot_mat2euler(R_mat)
            e *= rad2deg
            # TODO check if x-axis has to be negated
            t = np.array([p[0], p[1], -p[2], e[0], e[1], e[2]])
            euler_anim[frame_i][joint_name] = t
    return euler_anim


def get_local_transformations(animation, use_mmh_skeleton=False):
    # container for the result of this function
    local_animation = len(animation) * [None]

    ctop_dict = osim_child_to_parent
    if use_mmh_skeleton:
        ctop_dict = child_to_parent

    for i in range(len(animation)):
        local_animation[i] = {}
    # counter for the frame number
    frame_i = 0
    for joints in animation:
        # read each joint and transformation per frame
        for joint_name, transformation in joints.items():
            # extract the rotation from the orientation (see above)
            q = q_from_numpy(q=transformation[3:])
            # extract the joint position
            p = transformation[:3]
            if joint_name in ctop_dict:
                # compute the local orientation of this joint
                q_p, _ = get_parent_q(joints=joints, name=joint_name, ctop_dict=ctop_dict)
                q_local = q_p.inverse * q 
                #q_local = q * q_p.inverse # does not work
                #q_local = q.inverse * q_p # does not work
                # compute the local position of this joint
                # extract the parent joint position
                p_p, _ = get_parent_p(joints=joints, name=joint_name, ctop_dict=ctop_dict)
                p_local = p - p_p
            else:
                # if this particular joint has no parent joint, then return the identity (this is usually the Hip-Joint)
                q_local = Quaternion(q)
                p_local = p
            # reconstruct the local transformation
            t_local = np.array([
                p_local[0], p_local[1], p_local[2], 
                q_local.x, q_local.y, q_local.z, q_local.w])
            local_animation[frame_i][joint_name] = t_local
            # increment the frame counter
        frame_i += 1
    return local_animation


def mmh2mot(animation, durations):
    joint_anim = {}
    for joint_name, b_tuples in skeleton_angles.items():
        for b_tuple in b_tuples:
            joint_anim[b_tuple[0]] = np.zeros((len(animation), ))
    joint_anim["pelvis_tx"] = np.zeros((len(animation), ))
    joint_anim["pelvis_ty"] = np.zeros((len(animation), ))
    joint_anim["pelvis_tz"] = np.zeros((len(animation), ))

    # counter for the frame number
    frame_i = 0
    # variable to store the current absolute time which is calculated from the frame durations
    time = 0
    # absolute duration of the animation
    duration_anim = np.sum(durations)
    # container to store the normalized time values which we will write to the .mot file
    n_times = []
    for joints in animation:
        # read each joint and transformation per frame
        for joint_name, transformation in joints.items():
            # calculate angles only for defined joints
            if joint_name not in skeleton_angles:
                continue
            # extract the rotation from the orientation (see above)
            q = q_from_numpy(q=transformation[3:])
            
            if joint_name == "HIPS":
                joint_anim["pelvis_tx"][frame_i] = transformation[0]
                joint_anim["pelvis_ty"][frame_i] = transformation[1]
                joint_anim["pelvis_tz"][frame_i] = transformation[2]

            if joint_name in child_to_parent:
                q_p, parent_name = get_parent_q(joints=joints, name=joint_name)
                q_local = q_p.inverse * q
            else:
                # if this particular joint has no parent joint, then return the identity
                parent_name = ""
                q_local = Quaternion(q)
            for body_tuple in skeleton_angles[joint_name]:
                # extract the body tuple
                dof_name = body_tuple[0] 
                # plane normal
                plane_n = body_tuple[1]
                # default deviation vector of this joint
                default_vec = body_tuple[2]
                axis_idx = body_tuple[3]
                q_idx = body_tuple[4]
                e_idx = body_tuple[5]
                
                angle = 0
                if joint_name == "HIPS":
                    if dof_name == "pelvis_rotation":
                        vz = rotate(quat=q_local, vec=ax_z)
                        angle = angle_vector(a=ax_z, b=vz)
                    else:
                        angle, _, _ = get_angle(joint_name=joint_name, dof_name=dof_name, q_local=q_local, q_idx=q_idx, default_vec=default_vec, plane_n=plane_n, axis_idx=axis_idx) 
                    joint_anim[dof_name][frame_i] = angle 
                    continue
                elif joint_name == "SPINE":
                    if dof_name == "lumbar_extension":
                        angle_spine = get_in_between(joints=joints, joint_name=joint_name, ctop_dict=child_to_parent)
                        angle_chest = get_in_between(joints=joints, joint_name="CHEST", ctop_dict=child_to_parent)
                        angle = angle_spine + angle_chest 
                        joint_anim[dof_name][frame_i] = angle 
                    continue
                if e_idx == trans_idx:
                    continue
                if joint_name not in skeleton:
                    continue
                if joint_name.startswith("FOREARM") or joint_name.startswith("LEG") or joint_name.startswith("FOOT"):
                    angle = get_in_between(joints=joints, joint_name=joint_name, ctop_dict=child_to_parent)
                else:
                    angle = get_flex_add(joints=joints, joint_name=joint_name, idx=e_idx)
                joint_anim[dof_name][frame_i] = angle 
        # current absolute time according to the i-th frame
        time += durations[frame_i]
        # normalized time
        n_time = time / duration_anim
        # store the normalized time
        n_times.append(n_time)
        # increment the frame counter
        frame_i += 1
    return joint_anim, n_times


def angle_ops(joint_anim_cp):
    joint_anim_cp = negate(dof_name="lumbar_extension", joint_anim=joint_anim_cp)
    joint_anim_cp = negate(dof_name="knee_angle_r", joint_anim=joint_anim_cp)
    joint_anim_cp = negate(dof_name="knee_angle_l", joint_anim=joint_anim_cp)
    joint_anim_cp = add_offset(dof_name="ankle_angle_r", joint_anim=joint_anim_cp, offset=-45)
    joint_anim_cp = add_offset(dof_name="ankle_angle_l", joint_anim=joint_anim_cp, offset=-45)