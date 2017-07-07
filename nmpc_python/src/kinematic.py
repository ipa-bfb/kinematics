#!/usr/bin/python

import numpy as np
import PyKDL as kdl
import rospy

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

    def __init__(self, urdf, base_link, end_link):
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
        #print "kin_tree_:",self.kin_chain

        #print "joints: ", self.joints_name_
        #print "links: ", self.links_name_
        #print "joint_type: ", self.joints_type_
        #print "joint_origin: ", self.create_homo_matrix("arm_wrist_1_joint")
        #print "joint_origin: ", self.create_homo_matrix("arm_wrist_3_joint")
        #print "dist_joints: ", self.compute_distance_between_joint("arm_wrist_1_joint", "arm_wrist_3_joint")

    ##
    # @return List of joint names in the urdf_model (kinematic_chain)
    def get_joints_name(self, links=False, fixed=False):
        return self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
    ##
    # @return List of joint type in the urdf_model (kinematic_chain)
    def get_joints_type(self, links=False, fixed=False):
        joints_type = []
        model_description = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
        for it in model_description:
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
        model_description = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        for it in model_description:
            joint = self.urdf_model_.joint_map[it]
            if joint.name == joint_name:
                angle =  joint.origin.rpy
                pose = joint.origin.xyz
                continue
        homo_matrix = tf.transformations.compose_matrix(angles=angle, translate=pose)
        #print homo_matrix
        return homo_matrix

    # still error how to calculate diff of two homo matrix
    def compute_distance_between_joint(self, joint_name1, joint_name2):
        q1 = self.create_homo_matrix(joint_name1)
        q2 = self.create_homo_matrix(joint_name2)
        diff_mat = np.array(q1) - np.array(q2)
        #print diff_mat
        diff_mod = np.mod(diff_mat, 2 * np.pi)
        diff_alt = diff_mod - 2 * np.pi
        print diff_mod
        print diff_alt



if __name__ == '__main__':
    create_kinematic_chain_from_robot_description()