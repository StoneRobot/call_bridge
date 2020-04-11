#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <std_msgs/Bool.h>
// detector
#include "hirop_msgs/detection.h"
#include "hirop_msgs/listDetector.h"
#include "hirop_msgs/listObject.h"
#include "hirop_msgs/ObjectArray.h"
// pickplace
#include "hirop_msgs/SetGenActuator.h"
#include "hirop_msgs/listGenerator.h"
#include "hirop_msgs/listActuator.h"
#include "hirop_msgs/ShowObject.h"
#include "hirop_msgs/RemoveObject.h"
#include "hirop_msgs/Pick.h"
#include "hirop_msgs/Place.h"
#include "hirop_msgs/MoveToPos.h"
#include "hirop_msgs/MoveToName.h"
#include "hirop_msgs/PickPlaceStop.h"

// gripper
#include "hirop_msgs/listGripper.h"
#include "hirop_msgs/SetGripper.h"
#include "hirop_msgs/connectGripper.h"
#include "hirop_msgs/disConnectGripper.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/StopGripper.h"

#include <iostream>
#include <cmath>

class CallBridge 
{
public:
    CallBridge(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group, std::string name);

private:
    ros::NodeHandle nh;
    // 处理语音信息
    ros::Subscriber action_sub;
    void actionDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
    // detection
    ros::ServiceClient detection_client;
    ros::ServiceClient list_detector_client;
    ros::ServiceClient list_object_client;
    ros::Subscriber pose_sub;
    void poseCallback(const hirop_msgs::ObjectArray::ConstPtr &msg);
    // pickplace
    ros::ServiceClient set_gen_actuator_client;
    ros::ServiceClient list_generator_client;
    ros::ServiceClient list_actuator_client;
    ros::ServiceClient show_object_client;
    ros::ServiceClient remove_object_client;
    ros::ServiceClient pick_client;
    ros::ServiceClient place_client;
    ros::ServiceClient move_to_pose_client;
    ros::ServiceClient mvoe_to_name_client;
    ros::ServiceClient pickPlace_stop_client;
    // gripper
    ros::ServiceClient set_gripper_client;
    ros::ServiceClient list_gripper_client;
    ros::ServiceClient connect_gripper_client;
    ros::ServiceClient dis_connect_gripper_client;
    ros::ServiceClient open_gripper_client;
    ros::ServiceClient close_gripper_client;
    ros::ServiceClient stop_gripper_client;
    // photo
    ros::Subscriber photo_sub;
    ros::Publisher pick_pose_pub;
    void photoCallback(const std_msgs::Bool::ConstPtr& msg);
    // init
    bool setGripper();
    bool setGenActuator();
    bool setDetector();
    bool placeObject(hirop_msgs::Place place_srv);

    void showObject(geometry_msgs::Pose pose);

    void pick(geometry_msgs::Pose pose);
    void place(geometry_msgs::Pose pose);
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);

    std::string PLANNING_GROUP;
    //MoveGroup类只需使用要控制和规划的规划组的名称即可轻松设置。
    moveit::planning_interface::MoveGroupInterface& move_group;
    // 我们将使用规划场景接口类来添加和删除"虚拟世界"场景中的碰撞对象
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // 原始指针通常用于引用规划组以提高性能。
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    int intent = 1;
    int object = 1;
    int target = 1;
    int cnt = 0;
    int errorCnt = 0;
    bool command = false;
    int place_pose_flag = 0;
    hirop_msgs::Place place_pose1;
    hirop_msgs::Place place_pose2;
    hirop_msgs::Place place_pose3;
    hirop_msgs::MoveToName pose_name;

    std::vector<hirop_msgs::Place> place_poses;
    
    
};