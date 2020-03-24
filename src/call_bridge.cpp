#include "call_bridge/call_bridge.h"

CallBridge::CallBridge(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group):
move_group{group}
{
    nh = _n;
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::cout << "11" << std::endl;
    // move_group = group;
    std::cout << "11" << std::endl;
    hirop_msgs::SetGripper set;
    // set.gripperName = "hand";
    set.request.gripperName = "hand";
    set_gripper.call(set);
    std::cout << "11" << std::endl;
}

void CallBridge::actionDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    intent = msg->data[0];
    object = msg->data[1];
    target = msg->data[2];

    // hirop_msgs::detection det;
    // det.objectName = "s";
    // det.detectorName = "d";
    // det.detectorType = 1; // PYTHON
    // det.detectorConfig = "s";

    // detection_client.call(det);
}

void CallBridge::poseCallback(const hirop_msgs::ObjectArray::ConstPtr &msg)
{

    int i = msg->objects.size();
    // int i = 1;
    for(int j = 0; j < i; ++j)
    {
        // 转到object的方向
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        if(msg->objects[j].pose.pose.position.x)
            joint_group_positions[0] = msg->objects[j].pose.pose.position.y/msg->objects[j].pose.pose.position.x;
        else if(msg->objects[j].pose.pose.position.y > 0)
            joint_group_positions[0] = 1.57;
        else
            joint_group_positions[0] = 0;
        move_group.setJointValueTarget(joint_group_positions);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
            move_group.move();

        // 设置末端夹抓的方向
        geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
        target_pose.orientation = msg->objects[j].pose.pose.orientation;
        move_group.setPoseTarget(target_pose);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
            move_group.move();
         
        // 打开gripper
        hirop_msgs::openGripper op;
        open_gripper.call(op);
        // 直线去抓取object
        geometry_msgs::Pose target_pose2 = move_group.getCurrentPose().pose;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose2);
        target_pose2 = msg->objects[j].pose.pose;
        waypoints.push_back(target_pose2);
        move_group.setMaxVelocityScalingFactor(0.1);
        // 我们希望以 1 厘米的分辨率插值笛卡尔路径，
        // 这就是为什么我们将 0.01 指定为笛卡尔IK中的最大步骤。
        // 我们将跳转阈值指定为 0.0，从而有效地禁用它。
        // 警告 - 在操作实际硬件时禁用跳转阈值可能会
        // 导致冗余接头的大量不可预知运动，并且可能是一个安全问题
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        my_plan.trajectory_ = trajectory;
        move_group.execute(my_plan);
        // 关闭gripper
        close_gripper.call(op);
        // geometry_msgs::PoseStamped movePos;
        // movePos = msg->object[j].pose;
        // move_to_pose_client.call(movePos);
        
    }
}


