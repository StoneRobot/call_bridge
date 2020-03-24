#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <vector>
// detector
#include "hirop_msgs/detection.h"
#include "hirop_msgs/listDetector.h"
#include "hirop_msgs/listObject.h"
#include "hirop_msgs/ObjectArray.h"
// move to pos
#include "hirop_msgs/MoveToPos.h"
#include "hirop_msgs/SetGenActuator.h"
#include "hirop_msgs/listGenerator.h"

// gripper
#include "hirop_msgs/listGripper.h"
#include "hirop_msgs/SetGripper.h"
#include "hirop_msgs/connectGripper.h"
#include "hirop_msgs/disConnectGripper.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/StopGripper.h"


class CallBridge
{
public:
    CallBridge(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group);

private:
    ros::NodeHandle nh;

    ros::Subscriber action_sub = nh.subscribe("/action_data", 1, &CallBridge::actionDataCallback, this);
    void actionDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);

    ros::ServiceClient detection_client = nh.serviceClient<hirop_msgs::detection>("detection");
    ros::ServiceClient list_detector_client = nh.serviceClient<hirop_msgs::listDetector>("list_detector");
    ros::ServiceClient list_object_client = nh.serviceClient<hirop_msgs::listObject>("list_object");
    ros::Subscriber pose_sub = nh.subscribe("object_array", 1, &CallBridge::poseCallback, this);
    void poseCallback(const hirop_msgs::ObjectArray::ConstPtr &msg);

    ros::ServiceClient move_to_pose_client = nh.serviceClient<hirop_msgs::MoveToPos>("hsc3MoveTo");

    ros::ServiceClient set_gripper = nh.serviceClient<hirop_msgs::SetGripper>("setGripper");
    ros::ServiceClient list_gripper = nh.serviceClient<hirop_msgs::listGripper>("listGripper");
    ros::ServiceClient connect_gripper = nh.serviceClient<hirop_msgs::connectGripper>("connectGripper");
    ros::ServiceClient dis_connect_gripper = nh.serviceClient<hirop_msgs::disConnectGripper>("disConnectGripper");
    ros::ServiceClient open_gripper = nh.serviceClient<hirop_msgs::openGripper>("openGripper");
    ros::ServiceClient close_gripper = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper");
    ros::ServiceClient stop_gripper = nh.serviceClient<hirop_msgs::StopGripper>("stopGripper");

    ros::ServiceClient set_genActuator = nh.serviceClient<hirop_msgs::SetGenActuator>("setGenActuator");

    std::string PLANNING_GROUP = "panda_arm";
    //MoveGroup类只需使用要控制和规划的规划组的名称即可轻松设置。
    moveit::planning_interface::MoveGroupInterface& move_group;
    // 我们将使用规划场景接口类来添加和删除"虚拟世界"场景中的碰撞对象
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // 原始指针通常用于引用规划组以提高性能。
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    int intent;
    int object;
    int target;
    
};