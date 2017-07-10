#!/usr/bin/env python

import numpy as np

import PyKDL as kdl
import rospy

from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
import tf.transformations

def create_kdl_kin(base_link, end_link, urdf_filename=None, description_param="/robot_description"):
    if urdf_filename is None:
        robot = Robot.from_parameter_server(key=description_param)
    else:
        f = file(urdf_filename, 'r')
        robot = Robot.from_xml_string(f.read())
        f.close()
    return KDLKinematics(robot, base_link, end_link)


##
# Provides wrappers for performing KDL functions on a designated kinematic
# chain given a URDF representation of a robot.

class KDLKinematics(object):
    ##
    # Constructor
    # @param urdf URDF object of robot.
    # @param base_link Name of the root link of the kinematic chain.
    # @param end_link Name of the end link of the kinematic chain.
    # @param kdl_tree Optional KDL.Tree object to use. If None, one will be generated
    #                          from the URDF.
    def __init__(self, urdf, base_link, end_link, kdl_tree=None):
        if kdl_tree is None:
            [tree_ok, kdl_tree] = treeFromUrdfModel(urdf)

        if not tree_ok:
            rospy.logerr("KDLKinematics URDF tree parsing returned with error.")
            raise Exception("KDLKinematics URDF tree parsing returned with error.")

        self.tree = kdl_tree
        self.urdf = urdf


        base_link = base_link.split("/")[-1] # for dealing with tf convention
        end_link = end_link.split("/")[-1] # for dealing with tf convention
        self.chain = kdl_tree.getChain(base_link, end_link)

        self.base_link = base_link
        self.end_link = end_link

        # record joint information in easy-to-use lists
        self.joint_limits_lower = []
        self.joint_limits_upper = []
        self.joint_safety_lower = []
        self.joint_safety_upper = []
        self.joint_types = []
        self.joint_states = [0., 0., 0., 0., 0., 0.];
        rospy.Subscriber("/arm/joint_states", JointState, self.jointStateCallback)

        for jnt_name in self.get_joint_names():
            jnt = urdf.joint_map[jnt_name]

            if jnt.limit is not None:
                self.joint_limits_lower.append(jnt.limit.lower)
                self.joint_limits_upper.append(jnt.limit.upper)
            else:
                self.joint_limits_lower.append(None)
                self.joint_limits_upper.append(None)
            if jnt.safety_controller is not None:
                self.joint_safety_lower.append(jnt.safety_controller.soft_lower_limit)#.lower)
                self.joint_safety_upper.append(jnt.safety_controller.soft_upper_limit)#.upper)
            elif jnt.limit is not None:
                self.joint_safety_lower.append(jnt.limit.lower)
                self.joint_safety_upper.append(jnt.limit.upper)
            else:
                self.joint_safety_lower.append(None)
                self.joint_safety_upper.append(None)
            self.joint_types.append(jnt.type)
        def replace_none(x, v):
            if x is None:
                return v
            return x
        self.joint_limits_lower = np.array([replace_none(jl, -np.inf)
                                            for jl in self.joint_limits_lower])
        self.joint_limits_upper = np.array([replace_none(jl, np.inf)
                                            for jl in self.joint_limits_upper])
        self.joint_safety_lower = np.array([replace_none(jl, -np.inf)
                                            for jl in self.joint_safety_lower])
        self.joint_safety_upper = np.array([replace_none(jl, np.inf)
                                            for jl in self.joint_safety_upper])
        self.joint_types = np.array(self.joint_types)
        self.num_joints = len(self.get_joint_names())

        self._fk_kdl = kdl.ChainFkSolverPos_recursive(self.chain)
        self._ik_v_kdl = kdl.ChainIkSolverVel_pinv(self.chain)
        self._ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, self._fk_kdl, self._ik_v_kdl)
        self._jac_kdl = kdl.ChainJntToJacSolver(self.chain)
        self._dyn_kdl = kdl.ChainDynParam(self.chain, kdl.Vector.Zero())

    def extract_joint_state(self, js, joint_names=None):
        if joint_names is None:
            joint_names = self.get_joint_names()
        q   = np.zeros(len(joint_names))
        qd  = np.zeros(len(joint_names))
        eff = np.zeros(len(joint_names))
        for i, joint_name in enumerate(joint_names):
            js_idx = js.name.index(joint_name)
            if js_idx < len(js.position) and q is not None:
                q[i]   = js.position[js_idx]
            else:
                q = None
            if js_idx < len(js.velocity) and qd is not None:
                qd[i]  = js.velocity[js_idx]
            else:
                qd = None
            if js_idx < len(js.effort) and eff is not None:
                eff[i] = js.effort[js_idx]
            else:
                eff = None
        return q, qd, eff

    ##
    # @return List of link names in the kinematic chain.
    def get_link_names(self, joints=False, fixed=True):
        return self.urdf.get_chain(self.base_link, self.end_link, joints, fixed)

    ##
    # @return List of joint names in the kinematic chain.
    def get_joint_names(self, links=False, fixed=False):
        return self.urdf.get_chain(self.base_link, self.end_link,
                                   links=links, fixed=fixed)

    def get_joint_limits(self):
        return self.joint_limits_lower, self.joint_limits_upper

    def FK(self, q, link_number=None):
        if link_number is not None:
            end_link = self.get_link_names(fixed=False)[link_number]
        else:
            end_link = None
        homo_mat = self.forward(q, end_link)
        pos = tf.transformations.translation_from_matrix(q)
        quat = tf.transformations.quaternion_from_matrix(homo_mat)
        euler = tf.transformations.euler_from_matrix(homo_mat)
        return pos, quat, euler


    ##
    # Forward kinematics on the given joint angles, returning the location of the
    # end_link in the base_link's frame.
    # @param q List of joint angles for the full kinematic chain.
    # @param end_link Name of the link the pose should be obtained for.
    # @param base_link Name of the root link frame the end_link should be found in.
    # @return 4x4 numpy.mat homogeneous transformation
    def forward(self, q, end_link=None, base_link=None):
        link_names = self.get_link_names()
        if end_link is None:
            end_link = self.chain.getNrOfSegments()
            print end_link
        else:
            end_link = end_link.split("/")[-1]
            if end_link in link_names:
                end_link = link_names.index(end_link)
            else:
                print "Target segment %s not in KDL chain" % end_link
                return None
        if base_link is None:
            base_link = 0
        else:
            base_link = base_link.split("/")[-1]
            if base_link in link_names:
                base_link = link_names.index(base_link)
            else:
                print "Base segment %s not in KDL chain" % base_link
                return None
        base_trans = self._do_kdl_fk(q, base_link)

        if base_trans is None:
            print "FK KDL failure on base transformation."
        end_trans = self._do_kdl_fk(q, end_link)
        if end_trans is None:
            print "FK KDL failure on end transformation."
        return base_trans**-1 * end_trans

    def _do_kdl_fk(self, q, link_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self._fk_kdl.JntToCart(joint_list_to_kdl(q),
                                                   endeffec_frame,
                                                   link_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.mat([[M[0,0], M[0,1], M[0,2], p.x()],
                           [M[1,0], M[1,1], M[1,2], p.y()],
                           [M[2,0], M[2,1], M[2,2], p.z()],
                           [     0,      0,      0,     1]])
        else:
            return None


    ##
    # Returns the Jacobian matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return 6xN np.mat Jacobian
    # @param pos Point in base frame where the jacobian is acting on.
    #            If None, we assume the end_link
    def jacobian(self, q, pos=None):
        j_kdl = kdl.Jacobian(self.num_joints)
        q_kdl = joint_list_to_kdl(q)
        self._jac_kdl.JntToJac(q_kdl, j_kdl)
        if pos is not None:
            ee_pos = self.forward(q)[:3,3]
            pos_kdl = kdl.Vector(pos[0]-ee_pos[0], pos[1]-ee_pos[1],
                                  pos[2]-ee_pos[2])
            j_kdl.changeRefPoint(pos_kdl)
        return kdl_to_mat(j_kdl)

    ##
    # Returns the joint space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return NxN np.mat Inertia matrix
    def inertia(self, q):
        h_kdl = kdl.JntSpaceInertiaMatrix(self.num_joints)
        self._dyn_kdl.JntToMass(joint_list_to_kdl(q), h_kdl)
        return kdl_to_mat(h_kdl)

    ##
    # Returns the cartesian space mass matrix at the end_link for the given joint angles.
    # @param q List of joint angles.
    # @return 6x6 np.mat Cartesian inertia matrix
    def cart_inertia(self, q):
        H = self.inertia(q)
        J = self.jacobian(q)
        return np.linalg.inv(J * np.linalg.inv(H) * J.T)

    ##
    # Tests to see if the given joint angles are in the joint limits.
    # @param List of joint angles.
    # @return True if joint angles in joint limits.
    def joints_in_limits(self, q):
        lower_lim = self.joint_limits_lower
        upper_lim = self.joint_limits_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Tests to see if the given joint angles are in the joint safety limits.
    # @param List of joint angles.
    # @return True if joint angles in joint safety limits.
    def joints_in_safe_limits(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Clips joint angles to the safety limits.
    # @param List of joint angles.
    # @return np.array list of clipped joint angles.
    def clip_joints_safe(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.clip(q, lower_lim, upper_lim)

    ##
    # @return get joint angles
    def get_joint_angle(self):
        return self.joint_states

    # receives the joint states
    def jointStateCallback(self, msg):
        for i in range(0, len(msg.position)):
            self.joint_states[i] = msg.position[i]

    ##
    # Returns a set of random joint angles distributed uniformly in the safety limits.
    # @return np.array list of random joint angles.
    def random_joint_angles(self):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        lower_lim = np.where(np.isfinite(lower_lim), lower_lim, -np.pi)
        upper_lim = np.where(np.isfinite(upper_lim), upper_lim, np.pi)
        zip_lims = zip(lower_lim, upper_lim)
        return np.array([np.random.uniform(min_lim, max_lim) for min_lim, max_lim in zip_lims])

    ##
    # Returns a difference between the two sets of joint angles while insuring
    # that the shortest angle is returned for the continuous joints.
    # @param q1 List of joint angles.
    # @param q2 List of joint angles.
    # @return np.array of wrapped joint angles for retval = q1 - q2
    def difference_joints(self, q1, q2):
        diff = np.array(q1) - np.array(q2)
        diff_mod = np.mod(diff, 2 * np.pi)
        diff_alt = diff_mod - 2 * np.pi
        for i, continuous in enumerate(self.joint_types == 'continuous'):
            if continuous:
                if diff_mod[i] < -diff_alt[i]:
                    diff[i] = diff_mod[i]
                else:
                    diff[i] = diff_alt[i]
        return diff

def kdl_to_mat(m):
    mat =  np.mat(np.zeros((m.rows(), m.columns())))
    for i in range(m.rows()):
        for j in range(m.columns()):
            mat[i,j] = m[i,j]
    return mat

def joint_kdl_to_list(q):
    if q == None:
        return None
    return [q[i] for i in range(q.rows())]

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl


if __name__ == "__main__":

    rospy.init_node("kdl_kinematics")
    if not rospy.is_shutdown():
        create_kdl_kin("base_link", "arm_wrist_3_link")
        robot = Robot.from_parameter_server()
        kdl_kin = KDLKinematics(robot, "base_link", "arm_wrist_3_link")
        #q = kdl_kin.random_joint_angles()
        q = kdl_kin.get_joint_angle()
        #pose = kdl_kin.forward(q, "arm_wrist_3_link", "arm_wrist_2_link")
        pose = kdl_kin.forward(q)
        print pose
        kdl_kin.jacobian(q)

    else:
        rospy.logerr("Try to again connect ROS")

