#!/usr/bin/python

import numpy as np
import PyKDL as kdl
import rospy
from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
import tf.transformations

def create_kinematic_chain_from_robot_description(base_link="arm_base_link", end_link="arm_wrist_3_link", urdf_file=None, robot_description="/robot_description"):
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
    # Write in the list form
    def write_to_list(self):
        list_fk = []
        trans_wrt_origin = np.identity(4)

        for i, it in enumerate(self.joints_name_):
            trans = self.create_homo_matrix(it)
            trans_wrt_origin = np.dot(trans_wrt_origin, trans)
            list_fk.append((it, self.joints_type_[i], trans, trans_wrt_origin))
            #print trans_wrt_origin
        return list_fk

    # end_pose(On) final pose of end_link wrt to base or fix link
    def compute_jacobi(self, list_fk, end_pose=None):
        On = end_pose[:3,3]
        jacobi_mat = []

        for i in range(0,len(self.joints_name_)):
            mat_wrt_ori = list_fk[i][3]
            Zi = mat_wrt_ori[:3,2]

            if self.joints_type_[i] == 'revolute':
                Oi = mat_wrt_ori[:3,3]
                jacobi_mat.append((np.cross(Zi, On - Oi), Zi))

            elif self.joints_type_[i] == 'prismatic':
                jacobi_mat.append((Zi, [0.0, 0.0, 0.0]))

            else:
                rospy.loginfo("joint type is fixed")

#        # Acoord to ref solution, My Z is wrong so it gives wrong result
#        for i in range(-1,len(self.joints_name_)):
#
#            if i is not -1:
#                mat_wrt_ori = list_fk[i][3]
#                Zi = Z0
#                Z0 = mat_wrt_ori[:3,2]
#                #Z0 = [0, 1, 0]
#                Oi = O0
#                O0 = mat_wrt_ori[:3,3]
#                #print "zi: ",Zi
#                #print "Oi: ",Oi
#                jacobi_mat.append((np.cross(Zi, On - Oi), Zi))
#
#            if i is -1:
#                Z0 = [0 , 0, 1]
#                O0 = [0, 0 , 0]

        return jacobi_mat

if __name__ == '__main__':

    rospy.init_node('Forward_kinematics', anonymous=True)
    #create_kinematic_chain_from_robot_description()
    if not rospy.is_shutdown():
        robot = Robot.from_parameter_server()
        kdl_chain = KinematicChain(robot)
        pose = kdl_chain.write_to_list()
        #print "pose2: ",pose[2][3]
        jacobi_mat = kdl_chain.compute_jacobi(pose, end_pose=pose[5][3])  # means 5 joint element of list and 3 element of that joint)
        print np.squeeze(np.asarray(jacobi_mat))
    else:
        rospy.logerr("Try to connect ROS")
