#!/usr/bin/python

import numpy as np
import PyKDL as kdl
import rospy
from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
import tf.transformations

def create_kinematic_chain_from_robot_description(base_link="base_link", end_link="gripper", urdf_file=None, robot_description="/robot_description"):
    if urdf_file is None:
        robot = Robot.from_parameter_server(key=robot_description)
    else:
        fileObj = file(urdf_file, 'r')
        robot = Robot.from_xml_string(fileObj.read())
        fileObj.close()
    return KinematicChain(robot, base_link, end_link)


class KinematicChain(object):

    def __init__(self):
        rospy.loginfo("KinematicChain: create default constructor")

    def __init__(self, urdf, base_link="arm_base_link", end_link="arm_wrist_3_link"):
        [tree_ok, robot_tree_] = treeFromUrdfModel(urdf)

        if not tree_ok:
            rospy.logerr("KinematicChain: URDF tree is not parsed properly")
            raise Exception("KinematicChain: URDF tree is not parsed properly")

        self.robot_tree_ = robot_tree_
        self.urdf_model_ = urdf
        self.kin_chain_ = robot_tree_.getChain(base_link, end_link)
        self.base_link_ = base_link
        self.end_link_ = end_link

        self.joints_name_ = self.get_joints_name()
        self.joints_type_ = self.get_joints_type()
        self.links_name_ = self.get_links_name()
        self.num_joints_ = len(self.joints_name_)
        self.forward_kin_ = kdl.ChainFkSolverPos_recursive(self.kin_chain_)
        self.jacobi_ = kdl.ChainJntToJacSolver(self.kin_chain_)

        self.joint_states = [0., 0., 0., 0., 0., 0.];
        rospy.Subscriber("/arm/joint_states", JointState, self.jointStateCallback)

        #print endeffector_frame


        #print "kin_tree_:",self.kin_chain

        #print "joints: ", self.joints_name_
        #print "links: ", self.links_name_
        #print "joint_type: ", self.joints_type_
        #print "compute_transformation_between_joint: ", self.compute_transformation_between_joint("arm_wrist_1_joint", "arm_wrist_3_joint")

        #print self.forward_kinematics("arm_wrist_3_link", "arm_wrist_2_link")
        #print self.forward_kinematics()

    ##
    # @return List of joint names in the urdf_model (kinematic_chain)
    def get_joints_name(self, links=False, fixed=False):
        return self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
    ##
    # @return List of joint type in the urdf_model (kinematic_chain)
    def get_joints_type(self, links=False, fixed=False):
        joints_type = []
        joint_names = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
        for it in joint_names:
            joint = self.urdf_model_.joint_map[it]
            joints_type.append(joint.type)

        return joints_type

    ##
    # @return List of links names in the urdf_model (kinematic_chain)
    def get_links_name(self, joints=False, fixed=True):
        links_name = []
        links_name = self.urdf_model_.get_chain(self.base_link_, self.end_link_, joints=joints, fixed=fixed)
        return links_name

    ##
    # @param Name of joint of which transformation needed
    # @return 4x4 homogeneous transformation
    def create_homo_matrix(self, joint_name):
        joint_names = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        for it in joint_names:
            joint = self.urdf_model_.joint_map[it]
            if joint.name == joint_name:
                angle =  joint.origin.rpy
                pose = joint.origin.xyz
                continue
        homo_matrix = tf.transformations.compose_matrix(angles=angle, translate=pose)
        return homo_matrix

    ##
    # @return get joint angles
    def get_joint_angle(self):
        return self.joint_states

    # receives the joint states
    def jointStateCallback(self, msg):
        for i in range(0, len(msg.position)):
            self.joint_states[i] = msg.position[i]


    ##
    # Forward kinematics on the given joint angles, returning the location of the
    # end_link in the base_link's frame.
    # @param end_link Name of the link the pose should be obtained for.
    # @param base_link Name of the root link frame the end_link should be found in.
    # @return 4x4 numpy.mat homogeneous transformation
    def forward_kinematics(self,q, end_link=None, base_link=None):
        #q = self.get_joint_angle()

        link_names = self.links_name_
        if end_link is None:
            end_link = self.kin_chain_.getNrOfSegments()
        else:
            if end_link in link_names:
                end_link = link_names.index(end_link)
            else:
                print "Target segment %s not in KDL chain" % end_link
                return None

        if base_link is None:
            base_link = 0
        else:
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


    ##
    # @param Name of joint of which transformation needed
    # @return 4x4 homogeneous transformation
    def _do_kdl_fk(self, q, joint_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self.forward_kin_.JntToCart(self.create_joint_list(q),endeffec_frame ,joint_number)
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
    # @param q Name of joint of which transformation needed
    def create_joint_list(self, q):
        if q is None:
            return None
        if type(q) == np.matrix and q.shape[1] == 0:
            q = q.T.tolist()[0]
        q_kdl = kdl.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl

    ##
    # Write in the list form
    def write_to_list(self, q):
        joint_names = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        list_fk = []
        print joint_names

        for it in joint_names:
            joint = self.urdf_model_.joint_map[it]
            trans = self.forward_kinematics(q, joint.child, joint.parent)
            trans_wrt_origin = self.forward_kinematics(q, end_link=joint.child)
            list_fk.append((joint.name, joint.type, trans, trans_wrt_origin))

        return list_fk

    def compute_linear_Jacobian(self, list_fk):
        lin_jacobi = []
        for i in range (0, len(list_fk)):
            info_joint = list_fk[i]
            info_end_joint = list_fk[len(list_fk)-1]
            mat_wrt_ori = info_joint[2]
            end_mat_wrt_ori = info_end_joint[2]
            z = np.squeeze(np.asarray(mat_wrt_ori[:3,2]))
            Oi = mat_wrt_ori[:3,3]
            On = end_mat_wrt_ori[:3,3]
            diff = np.squeeze(np.asarray(On - Oi))
            print np.cross(z, diff)
#            print lin_jacobi


    def compute_jacobian(self, q):
        joint_names = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        list_fk = []
        end_joint_name = joint_names[self.num_joints_-1]    # get child link name of end_link
        joint_end = self.urdf_model_.joint_map[end_joint_name]
        joint_start = self.urdf_model_.joint_map[joint_names[0]]
        i = 0

        for it in joint_names:
            joint = self.urdf_model_.joint_map[it]

            if i is not 0:

                #if joint.type is 'fixed':
                #    rospy.loginfo("Type of joint is fixed")
                #    continue

                #elif joint.type is 'revolute':
                #rospy.loginfo("Type of joint is revolute")
                z_i = np.squeeze(np.asarray( self.forward_kinematics(q, joint.child, joint_start.parent)[:3,2]))
                o_i = np.squeeze(np.asarray(self.forward_kinematics(q, joint.child, joint_start.parent)[:3, 3]))
                o_n = np.squeeze(np.asarray( self.forward_kinematics(q, joint_end.child, joint_start.parent)[:3, 3] ))
                print "child: ", joint.child
                print "joint_end_child: ", joint_end.child
                print "joint_start_parent: ", joint_start.parent
                print o_i
                print "z_i: ", z_i
                print "diff: ",np.cross(z_i, o_n-o_i)

                #else:
                #    rospy.loginfo("Type of joint is other")

            else:
                z0 = [0.0, 0.0, 1.0]
                #z0 = np.squeeze(np.asarray( self.forward_kinematics(q, joint.child, joint.parent)[:3,2]))
                o_i = np.squeeze(np.asarray(self.forward_kinematics(q, joint.child, joint_start.parent)[:3, 3]))
                o_n = np.squeeze(np.asarray(self.forward_kinematics(q, joint_end.child, joint_start.parent)[:3, 3]))
                print z0
                print o_n
                print o_i
                print o_n - o_i
                print "diff0: ", np.cross(z0, o_n - o_i)

            i = i + 1

    def jacobi(self, q, pose=None):
        joint_names = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        joint_end = self.urdf_model_.joint_map[joint_names[self.num_joints_-1]] # get child link name of end_link

        for it in joint_names:
            joint = self.urdf_model_.joint_map[it]

            z_i = np.squeeze(np.asarray( self.forward_kinematics(q, joint.child, joint.parent)[:3,2]))
            theta_i = np.squeeze(np.asarray(self.forward_kinematics(q, joint.child, joint.parent)[:3, 3]))
            #theta_n = np.squeeze(np.asarray( self.forward_kinematics(q, joint_end.child, joint.parent)[:3, 3] ))
            theta_n = np.squeeze(np.asarray(pose[:3,3]))
            print "z_i: ",z_i
            #print "theta_i: ",theta_i
            #print "theta_n: ",theta_n
            print "diff: ",np.cross(z_i, theta_n-theta_i)



    def jacobi_calculation(self, q, pose=None):

        j_kdl = kdl.Jacobian(self.num_joints_)
        q_kdl = self.create_joint_list(q)
        self.jacobi_.JntToJac(q_kdl, j_kdl)
        if pose is not None:
            ee_pose = self.forward_kinematics(q)[:3,3]
            pos_kdl = kdl.Vector(pose[0] - ee_pose[0], pose[1] - ee_pose[1], pose[2] - ee_pose[2])
            j_kdl.changeRefPoint(pos_kdl)
        return j_kdl



if __name__ == '__main__':

    rospy.init_node('Forward_kinematics', anonymous=True)
    #create_kinematic_chain_from_robot_description()
    if not rospy.is_shutdown():
        robot = Robot.from_parameter_server()
        kdl_chain = KinematicChain(robot)
        #q = kdl_chain.get_joint_angle()
        q = [0., 0., 0., 0., 0., 0.]
        #pose = kdl_chain.forward_kinematics(q, 'arm_wrist_3_link', 'arm_forearm_link')
        #print "pose: ", pose
        pose1 = kdl_chain.create_homo_matrix('arm_wrist_3_joint')
        pose2 = kdl_chain.create_homo_matrix('arm_wrist_2_joint')
        pose3 = kdl_chain.create_homo_matrix('arm_wrist_1_joint')
        pose_f = np.dot(pose3, pose2)
        print np.dot(pose_f, pose1)
        #list_fk = kdl_chain.write_to_list(q)
        #pose = list_fk[5]
        #print pose[2]

        #kdl_chain.compute_jacobian(q)
        #print kdl_chain.jacobi(q, pose)
    else:
        rospy.logerr("Try to connect ROS")
