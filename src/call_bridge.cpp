#include "call_bridge/call_bridge.h"


CallBridge::CallBridge(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group, std::string name):
move_group{group},
PLANNING_GROUP{name}
{
    nh = _n;
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // move_group.setPoseReferenceFrame("base_link");
    move_group.allowReplanning(true);
    ROS_INFO_NAMED("robot frame_id is %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("robot end eff is %s", move_group.getEndEffectorLink().c_str());

    action_sub = nh.subscribe("/action_data", 1, &CallBridge::actionDataCallback, this);
    // detection
    detection_client = nh.serviceClient<hirop_msgs::detection>("detection");
    list_detector_client = nh.serviceClient<hirop_msgs::listDetector>("list_detector");
    list_object_client = nh.serviceClient<hirop_msgs::listObject>("list_object");
    pose_sub = nh.subscribe("object_array", 1, &CallBridge::poseCallback, this);
    // pickplace
    set_gen_actuator_client = nh.serviceClient<hirop_msgs::SetGenActuator>("setGenActuator");
    list_generator_client = nh.serviceClient<hirop_msgs::listGenerator>("listGenerator");
    list_actuator_client = nh.serviceClient<hirop_msgs::listActuator>("listActuator");
    show_object_client = nh.serviceClient<hirop_msgs::ShowObject>("showObject");
    remove_object_client = nh.serviceClient<hirop_msgs::RemoveObject>("removeObject");
    pick_client = nh.serviceClient<hirop_msgs::Pick>("pick");
    place_client = nh.serviceClient<hirop_msgs::Place>("place");
    move_to_pose_client = nh.serviceClient<hirop_msgs::MoveToPos>("moveToPos");
    mvoe_to_name_client = nh.serviceClient<hirop_msgs::MoveToName>("moveToFoName");
    pickPlace_stop_client = nh.serviceClient<hirop_msgs::PickPlaceStop>("pickplaceStop");
    // gripper
    set_gripper_client = nh.serviceClient<hirop_msgs::SetGripper>("setGripper");
    list_gripper_client = nh.serviceClient<hirop_msgs::listGripper>("listGripper");
    connect_gripper_client = nh.serviceClient<hirop_msgs::connectGripper>("connectGripper");
    dis_connect_gripper_client = nh.serviceClient<hirop_msgs::disConnectGripper>("disConnectGripper");
    open_gripper_client = nh.serviceClient<hirop_msgs::openGripper>("openGripper");
    close_gripper_client = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper");
    stop_gripper_client = nh.serviceClient<hirop_msgs::StopGripper>("stopGripper");
    // photo
    photo_sub = nh.subscribe("photo", 1, &CallBridge::photoCallback, this);
    pick_pose_pub = nh.advertise<geometry_msgs::Pose>("pick_pose", 1);

    // init
    // if(setGripper())
    //     ROS_INFO("set gripper succee");
    // else
    //     ROS_INFO("set gripper failed");
    ROS_INFO_STREAM("set gripper "<< (setGripper() ? "Succeed" : "Faild"));
    if(setGenActuator())
        ROS_INFO("set Gen Actuator success");
    else
        ROS_INFO("set Gen Actuator failed");

    if(setDetector())
        ROS_INFO("set detector succee");
    else
        ROS_INFO("set detector failed");

    tf2::Quaternion orien;
    orien.setRPY(0, 0, -1.57);
    

    place_pose1.request.placePos.header.frame_id = "base_link";
    place_pose1.request.placePos.pose.position.x = 0.418;
    place_pose1.request.placePos.pose.position.y = -0.68;
    place_pose1.request.placePos.pose.position.z = 0.63;
    place_pose1.request.placePos.pose.orientation = tf2::toMsg(orien);

    place_pose2.request.placePos.header.frame_id = "base_link";
    place_pose2.request.placePos.pose.position.x = 0.418;
    place_pose2.request.placePos.pose.position.y = -0.68;
    place_pose2.request.placePos.pose.position.z = 0.32;
    place_pose2.request.placePos.pose.orientation = tf2::toMsg(orien);

    place_pose3.request.placePos.header.frame_id = "base_link";
    place_pose3.request.placePos.pose.position.x = 0.8;
    place_pose3.request.placePos.pose.position.y = 0;
    place_pose3.request.placePos.pose.position.z = 0.25;
    place_pose3.request.placePos.pose.orientation.x = 0;
    place_pose3.request.placePos.pose.orientation.y = 0;
    place_pose3.request.placePos.pose.orientation.z = 0;
    place_pose3.request.placePos.pose.orientation.w = 1;

    place_poses.push_back(place_pose1);
    place_poses.push_back(place_pose2);
    place_poses.push_back(place_pose3);


    pose_name.request.PosName = "home";
}

bool CallBridge::setGripper()
{

    hirop_msgs::SetGripper set_gripper_srv;
    hirop_msgs::connectGripper connect_gripper_srv;
    set_gripper_srv.request.gripperName = "SerialGripper";
    this->set_gripper_client.call(set_gripper_srv);
    this->connect_gripper_client.call(connect_gripper_srv);
    return set_gripper_srv.response.isSucceeful;

    return false;
}

bool CallBridge::setGenActuator()
{
    hirop_msgs::listGenerator list_generator_srv;
    hirop_msgs::listActuator list_actuator_srv;
    hirop_msgs::SetGenActuator set_gen_actuator_srv;
    if(this->list_generator_client.call(list_generator_srv) && this->list_actuator_client.call(list_actuator_srv))
    {
        set_gen_actuator_srv.request.generatorName = list_generator_srv.response.generatorList[0];
        set_gen_actuator_srv.request.actuatorName = list_actuator_srv.response.actuatorList[0];
        if(this->set_gen_actuator_client.call(set_gen_actuator_srv))
            return true;
    }
    return false;
}

bool CallBridge::setDetector()
{
    return false;
}

void CallBridge::actionDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    intent = msg->data[0];
    object = msg->data[1];
    target = msg->data[2];

    hirop_msgs::detection det;
    det.request.objectName = "object";
    det.request.detectorName = "detector";
    det.request.detectorType = 1;
    det.request.detectorConfig = "config";
    if(detection_client.call(det))
    {
        ROS_INFO("Identify the successful");
    }
}

// void CallBridge::orientationConstraint(moveit_msgs::OrientationConstraint ocm)
// {
//     moveit_msgs::Constraints orientation_constraints;
//     orientation_constraints.orientation_constraints.push_back(ocm);
//     this->move_group.setPathConstraints(orientation_constraints);
//     robot_state::RobotState start_state(*move_group.getCurrentState());
//     // geometry_msgs::Pose start_pose;
//     // start_pose = this->move_group.getCurrentPose("link6").pose;
//     // this->move_group.setStartState(start_pose);
//     // this->move_group.setPlanningTime(10.0);
// }

// void CallBridge::clearPathConstraints()
// {
//     this->move_group.clearPathConstraints();
//     // this->move_group.setStartStateToCurrentState();
// }

void CallBridge::showObject(geometry_msgs::Pose pose)
{
    hirop_msgs::ShowObject srv;
    // tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, -M_PI / 2);
    // ROS_INFO_STREAM(tf2::toMsg(orientation));
    srv.request.objPose.header.frame_id = "base_link";
    srv.request.objPose.pose.position.x = pose.position.x;
    srv.request.objPose.pose.position.y = pose.position.y;
    srv.request.objPose.pose.position.z = pose.position.z;
    srv.request.objPose.pose.orientation.x = pose.orientation.x;
    srv.request.objPose.pose.orientation.y = pose.orientation.y;
    srv.request.objPose.pose.orientation.z = pose.orientation.z;
    srv.request.objPose.pose.orientation.w = pose.orientation.w;
    // srv.request.objPose.pose.orientation = tf2::toMsg(orientation);
    if(show_object_client.call(srv))
    {
        ROS_INFO_STREAM("show object "<< (srv.response.isSetFinsh ? "Succeed" : "Faild"));
    }
    else
    {
        ROS_INFO("check \\showObject service ");
    }
}

void CallBridge::poseCallback(const hirop_msgs::ObjectArray::ConstPtr &msg)
{
    nh.getParam("/call_bridge/intent", intent);
    nh.getParam("/call_bridge/target", target);
    nh.getParam("/call_bridge/effective_command", command);
    move_group.setPlanningTime(10.0);
    move_group.allowReplanning(true);
    move_group.setMaxVelocityScalingFactor(0.2);
    int i = msg->objects.size();
    // if(command == true)
    for(int j = 0; j < i; ++j)
    {    
        // 显示object
        cnt ++;
        showObject(msg->objects[j].pose.pose);
        // 获取参数
        nh.getParam("/call_bridge/intent", intent);
        nh.getParam("/call_bridge/target", target);
        if(intent == 1)
        {
            geometry_msgs::Pose pose;
            pose = msg->objects[0].pose.pose;
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, -1.57);
            pose.orientation = tf2::toMsg(orientation);
            pick(pose);
            ros::WallDuration(1.0).sleep();
            this->mvoe_to_name_client.call(pose_name);
        }

            // 直线去抓取object
            // 从现在位置
            move_group.setPoseReferenceFrame("base_link");
            geometry_msgs::Pose target_pose2 = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
            std::vector<geometry_msgs::Pose> waypoints;
            target_pose2.position.z -= 1.0;
            ROS_INFO_STREAM("target_pose2: " << target_pose2);
            waypoints.push_back(target_pose2);

            // 到预抓取位置
            // geometry_msgs::Pose target_finish = msg->objects[j].pose.pose;
            geometry_msgs::Pose target_finish = place_poses[target].request.placePos.pose;
            target_finish.position.x *= 0.85;
            target_finish.position.y *= 0.85;
            target_finish.position.z *= 1;

            target_finish.orientation.x = 0;
            target_finish.orientation.y = 0.254;
            target_finish.orientation.z = 0;
            target_finish.orientation.w = 0.967;

            waypoints.push_back(target_finish);
            ROS_INFO_STREAM("target_finish: " << target_finish);

            // 警告 - 在操作实际硬件时禁用跳转阈值可能会
            // 导致冗余接头的大量不可预知运动，并且可能是一个安全问题
            move_group.setStartStateToCurrentState();
            moveit_msgs::RobotTrajectory trajectory;
            double jump_threshold = 0.0;
            double eef_step = 0.02;
            double fraction = 0;
            int cnt = 0;
            while (fraction < 1.0 && cnt < 100)
            {
                fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                cnt ++;
            }
            ROS_INFO_STREAM( "waypoints "<<waypoints.size()<<" "<<fraction);
            my_plan.trajectory_ = trajectory;
            // 运动
            move_group.execute(my_plan);
            // to pick
            hirop_msgs::Pick pick;
            pick.request.pickPos = msg->objects[j].pose;
            // 自定义
            // this->pick(pick);
            // 调用桥
            if(intent == 0)
            if(pick_client.call(pick))
            {
                ROS_INFO_STREAM("pick "<< (pick.response.isPickFinsh ? "Succeed" : "Faild"));
            }
            else
            {
                ROS_INFO("check \\pick service");
                // this->mvoe_to_name_client.call(pose_name);
                // return;
            }


        // back home
        // ROS_INFO("back home...");
        // this->mvoe_to_name_client.call(pose_name);
        // ROS_INFO("at home");

        // place
        hirop_msgs::Place place_pose;
        place_pose = place_poses[target];
        ROS_INFO_STREAM("place: " << place_pose.request.placePos.pose << " " << target);

        
        move_group.setPoseReferenceFrame("base_link");
        geometry_msgs::Pose t_pose;
        // t_pose = place_pose.request.placePos.pose;
        // ROS_INFO("to pose 1");
        // t_pose.position.x *= 0.85;
        // t_pose.position.y *= 0.85;
        // t_pose.position.z *= 1.05;
        // t_pose.orientation.x = 0;
        // t_pose.orientation.y = 0.254;
        // t_pose.orientation.z = 0;
        // t_pose.orientation.w = 0.967;
        // move_group.setPoseTarget(t_pose);
        // move_group.plan(my_plan);
        // move_group.move();

        ROS_INFO("to pose 2");
        t_pose = place_pose.request.placePos.pose;
        move_group.setPoseTarget(t_pose);
        move_group.plan(my_plan);
        move_group.move();

        // ROS_INFO("to pose 3");
        // t_pose.position.x *= 0.85;
        // t_pose.position.y *= 0.85;
        // t_pose.position.z *= 1.05;
        // t_pose.orientation.x = 0;
        // t_pose.orientation.y = 0.254;
        // t_pose.orientation.z = 0;
        // t_pose.orientation.w = 0.967;
        // move_group.setPoseTarget(t_pose);
        // move_group.plan(my_plan);
        // move_group.move();
        ROS_INFO("back_home");



        // 调用桥
        // if(!this->placeObject(place_pose))
        // {
        //     place_pose.request.placePos.pose = msg->objects[j].pose.pose;
        //     this->placeObject(place_pose);
        //     errorCnt ++;
        // }
        this->mvoe_to_name_client.call(pose_name);
        nh.setParam("/call_bridge/effective_command", false);
        nh.setParam("/call_bridge/over", true);
        nh.setParam("/call_bridge/cnt", cnt);
        nh.setParam("/call_bridge/error_cnt", errorCnt);
    }
}

void CallBridge::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger_1_joint";
  posture.joint_names[1] = "right_finger_1_joint";
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.6;
  posture.points[0].positions[1] = -0.6;  
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void CallBridge::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger_1_joint";
  posture.joint_names[1] = "right_finger_1_joint";
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;  
  posture.points[0].time_from_start = ros::Duration(0.5);

}

void CallBridge::pick(geometry_msgs::Pose pose)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  // 抓取姿态
  grasps[0].grasp_pose.header.frame_id = "base_link";
  // tf2::Quaternion orientation;
  // orientation.setRPY(0, 0, -M_PI / 2);
  // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.orientation.x = pose.orientation.x;
  grasps[0].grasp_pose.pose.orientation.y = pose.orientation.y;
  grasps[0].grasp_pose.pose.orientation.z = pose.orientation.z;
  grasps[0].grasp_pose.pose.orientation.w = pose.orientation.w;
  grasps[0].grasp_pose.pose.position.x = pose.position.x;
  grasps[0].grasp_pose.pose.position.y = pose.position.y;
  grasps[0].grasp_pose.pose.position.z = pose.position.z;
  // 发送pick的位置信息
  pick_pose_pub.publish(grasps[0].grasp_pose.pose);
  // 抓取方向
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";

  grasps[0].pre_grasp_approach.direction.vector.y = -1;

  grasps[0].pre_grasp_approach.min_distance = 0.055;
  grasps[0].pre_grasp_approach.desired_distance = 0.5;
  // 撤退方向
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";

  grasps[0].post_grasp_retreat.direction.vector.y = 1;

  grasps[0].post_grasp_retreat.min_distance = 0.20;
  grasps[0].post_grasp_retreat.desired_distance = 0.35;
  // 模拟关闭夹爪
  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);
  // 动作
  move_group.pick("object", grasps);
}

// void CallBridge::place(geometry_msgs::Pose pose, float pre_vec[], float back_vec[])
// {

//   std::vector<moveit_msgs::PlaceLocation> place_location;
//   place_location.resize(1);
//   // 抓取位置
//   place_location[0].place_pose.header.frame_id = "base_link";
//   // tf2::Quaternion orientation;
//   // orientation.setRPY(0, 0, 0);
//   // place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
//   place_location[0].place_pose.pose.orientation.x = pose.orientation.x;
//   place_location[0].place_pose.pose.orientation.y = pose.orientation.y;
//   place_location[0].place_pose.pose.orientation.z = pose.orientation.z;
//   place_location[0].place_pose.pose.orientation.w = pose.orientation.w;
//   place_location[0].place_pose.pose.position.x = pose.position.x;
//   place_location[0].place_pose.pose.position.y = pose.position.y;
//   place_location[0].place_pose.pose.position.z = pose.position.z;
//   // 发送放置的位置信息
//   place_pose_pub.publish(place_location[0].place_pose.pose);
//   // 放置方向
//   place_location[0].pre_place_approach.direction.header.frame_id = "base_link";

// // place_location[0].pre_place_approach.direction.vector.x = 1;
// //   place_location[0].pre_place_approach.direction.vector.x = 1;
// //   place_location[0].pre_place_approach.direction.vector.y = pre_vec[1];
// // 不同的
//     if(intent == 0)
//     {
//         place_location[0].pre_place_approach.direction.vector.y = -1; 
//         place_location[0].post_place_retreat.direction.vector.y = 1;
//     }
//     else if(intent == 1)
//     {
//         place_location[0].pre_place_approach.direction.vector.z = -1; 
//         place_location[0].post_place_retreat.direction.vector.z = 1;
//     }  
//   place_location[0].pre_place_approach.min_distance = 0.095;
//   place_location[0].pre_place_approach.desired_distance = 0.115;
//   // 撤退方向
//   place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
// //   place_location[0].post_place_retreat.direction.vector.x = back_vec[0];
// //   place_location[0].post_place_retreat.direction.vector.y = back_vec[1];

        
//   place_location[0].post_place_retreat.min_distance = 0.1;
//   place_location[0].post_place_retreat.desired_distance = 0.25;
//   // 模拟打开夹爪
//   openGripper(place_location[0].post_place_posture);
//   // 抓取动作
//   move_group.place("object", place_location);
// }

bool CallBridge::placeObject(hirop_msgs::Place place_srv)
{
    if(this->place_client.call(place_srv))
    {
        ROS_INFO_STREAM("place "<< (place_srv.response.isPlaceFinsh ? "Succeed" : "Faild"));
        return true;
    }
    else
    {
        ROS_INFO("check \\place service");
        return false;
    }
}

void CallBridge::photoCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        ROS_INFO("slow");
        move_group.setMaxVelocityScalingFactor(0.01);
        move_group.setMaxAccelerationScalingFactor(0.01);
    }
    else
    {
        ROS_INFO("fast");
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
    }
}


