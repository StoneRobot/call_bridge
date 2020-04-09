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
    place_pose2.request.placePos.pose.position.z = 0.30;
    place_pose2.request.placePos.pose.orientation = tf2::toMsg(orien);

    place_pose3.request.placePos.header.frame_id = "base_link";
    place_pose3.request.placePos.pose.position.x = 0.8;
    place_pose3.request.placePos.pose.position.y = 0;
    place_pose3.request.placePos.pose.position.z = 0.25;
    place_pose3.request.placePos.pose.orientation.x = 0;
    place_pose3.request.placePos.pose.orientation.y = 0;
    place_pose3.request.placePos.pose.orientation.z = 0;
    place_pose3.request.placePos.pose.orientation.w = 1;

    // place_pose3.request.placePos.pose


    // place_pose2.request.placePos.pose.orientation.x = 0.0;
    // place_pose2.request.placePos.pose.orientation.y = 0.0;
    // place_pose2.request.placePos.pose.orientation.z = 0.0;
    // place_pose2.request.placePos.pose.orientation.w = 1;


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

void CallBridge::orientationConstraint(moveit_msgs::OrientationConstraint ocm)
{
    moveit_msgs::Constraints orientation_constraints;
    orientation_constraints.orientation_constraints.push_back(ocm);
    this->move_group.setPathConstraints(orientation_constraints);
    robot_state::RobotState start_state(*move_group.getCurrentState());
    // geometry_msgs::Pose start_pose;
    // start_pose = this->move_group.getCurrentPose("link6").pose;
    // this->move_group.setStartState(start_pose);
    // this->move_group.setPlanningTime(10.0);
}

void CallBridge::clearPathConstraints()
{
    this->move_group.clearPathConstraints();
    // this->move_group.setStartStateToCurrentState();
}

void CallBridge::poseCallback(const hirop_msgs::ObjectArray::ConstPtr &msg)
{

    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalPositionTolerance(0.01);
    move_group.allowReplanning(true);
    move_group.setMaxVelocityScalingFactor(1);
    int i = msg->objects.size();
    for(int j = 0; j < i; ++j)
    {    
        // show object
        hirop_msgs::ShowObject show_object_srv;
        show_object_srv.request.objPose = msg->objects[j].pose;
        if(this->show_object_client.call(show_object_srv))
        {
            ROS_INFO_STREAM("show object "<< (show_object_srv.response.isSetFinsh ? "Succeed" : "Faild"));
        }
        else
        {
            ROS_INFO("check \\showObject service ");
        }

        nh.getParam("/call_bridge/intent", intent);
        if(intent == 1)
        {
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            //
            // 接下来获取规划组的当前关节角度值数据集。
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
            // 现在，让我们修改一个关节，计划到新的关节空间目标，并可视化计划。
            // float tan = (msg->objects[j].pose.pose.position.y/msg->objects[j].pose.pose.position.x);
            joint_group_positions[0] = 0;
            joint_group_positions[1] = -1.91;
            joint_group_positions[2] = 3.45;
            joint_group_positions[3] = -1.51;
            joint_group_positions[4] = 1.48;
            joint_group_positions[5] = 0; 
            ROS_INFO_STREAM(joint_group_positions[0]);
            move_group.setJointValueTarget(joint_group_positions);
            auto success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(success)
                move_group.move();
            moveit_msgs::OrientationConstraint ocm;
            ocm.orientation = this->move_group.getCurrentPose("link6").pose.orientation;
            ocm.header.frame_id = "base_link";
            ocm.link_name = "link6";
            ocm.absolute_x_axis_tolerance = 0.01;
            ocm.absolute_y_axis_tolerance = 0.01;
            ocm.absolute_z_axis_tolerance = 0.01;
            ocm.weight = 1;
            orientationConstraint(ocm);
            // geometry_msgs::PoseStamped pose1;
            // pose1 = msg->objects[j].pose;
            // pose1.pose.position.x *= 0.8;
            // pose1.pose.position.y *= 0.8;
            // // pose1.pose.position.z
            // move_group.setPoseTarget(pose1);
            // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            // if(success)
            //     move_group.move();
        }

        if(intent == 0)
        {

            // 直线去抓取object
            // 从现在位置
            move_group.setPoseReferenceFrame("base_link");
            geometry_msgs::Pose target_pose2 = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
            std::vector<geometry_msgs::Pose> waypoints;
            target_pose2.position.z -= 1.0;
            ROS_INFO_STREAM("target_pose2: " << target_pose2);
            waypoints.push_back(target_pose2);

            // 到预抓取位置
            geometry_msgs::Pose target_finish = msg->objects[j].pose.pose;
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
        }
        
        // to pick
        hirop_msgs::Pick pick;
        pick.request.pickPos = msg->objects[j].pose;
        if(pick_client.call(pick))
        {
            ROS_INFO_STREAM("pick "<< (pick.response.isPickFinsh ? "Succeed" : "Faild"));
        }
        else
        {
            ROS_INFO("check \\pick service");
            this->mvoe_to_name_client.call(pose_name);
            if(intent == 1)
                clearPathConstraints();
            return;
        }
        if(intent == 1)
            clearPathConstraints();
        // back home
        ROS_INFO("back home...");
        this->mvoe_to_name_client.call(pose_name);
        ROS_INFO("at home");

        // place
        hirop_msgs::Place place_pose;
        nh.getParam("/call_bridge/place_pose", place_pose_flag);
        if(place_pose_flag == 0)
        {
            place_pose = this->place_pose1;
        }
        else if(place_pose_flag == 1)
        {
            place_pose = this->place_pose2;
        }
        else if(place_pose_flag == 2)
        {
            place_pose = this->place_pose3;
        }
        ROS_INFO_STREAM("place: " << place_pose.request.placePos.pose << " " << place_pose_flag);
        // waypoints.clear();
        // geometry_msgs::Pose target2_pose = move_group.getCurrentPose(move_group.getEndEffectorLink().c_str()).pose;
        // target2_pose.position.z -= 1.0;
        // ROS_INFO_STREAM("target2_pose: " << target2_pose);
        // waypoints.push_back(target2_pose);

        // target2_pose = this->place_pose1.request.placePos.pose;
        // target2_pose.position.x *= 0.8;
        // target2_pose.position.y *= 0.8;
        // target2_pose.position.z *= 1.05;
        // ROS_INFO_STREAM("target2_pose: " << target2_pose);
        // waypoints.push_back(target2_pose);

        // eef_step = 0.001;
        // jump_threshold = 0.0;
        // fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        // my_plan.trajectory_ = trajectory;
        // ROS_INFO_STREAM( "waypoints "<<waypoints.size()<<" "<<fraction);
        // move_group.execute(my_plan);

        // geometry_msgs::Pose now_pose = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;

        // place_pose.request.placePos.pose.orientation = now_pose.orientation;

        // place(place_pose);
        // hirop_msgs::openGripper op;
        // this->open_gripper_client.call(op);
        if(!this->placeObject(place_pose))
        {
            place_pose.request.placePos.pose = pick.request.pickPos.pose;
            this->placeObject(place_pose);
        }

        this->mvoe_to_name_client.call(pose_name);
    }
}

void CallBridge::place(hirop_msgs::Place place_pose)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "base_link";

  place_location[0].place_pose.pose = place_pose.request.placePos.pose;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.x = -place_pose.request.placePos.pose.orientation.x;
  place_location[0].pre_place_approach.direction.vector.y = -place_pose.request.placePos.pose.orientation.y;
  place_location[0].pre_place_approach.direction.vector.z = -place_pose.request.placePos.pose.orientation.z;

  place_location[0].pre_place_approach.min_distance = 0.04;
  place_location[0].pre_place_approach.desired_distance = 0.215;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.x = place_pose.request.placePos.pose.orientation.x;
  place_location[0].post_place_retreat.direction.vector.y = place_pose.request.placePos.pose.orientation.y;
  place_location[0].post_place_retreat.direction.vector.z = place_pose.request.placePos.pose.orientation.z;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
//   openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
//   move_group.setSupportSurfaceName("shelf");
  // Call place to place the object using the place locations given.
  move_group.place("object", place_location);
  // END_SUB_TUTORIAL
}


void CallBridge::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(4);
  posture.joint_names[0] = "left_finger_1_link";
  posture.joint_names[1] = "left_finger_2_link";
  posture.joint_names[2] = "right_finger_1_link";
  posture.joint_names[3] = "right_finger_2_link";
  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(4);
  posture.points[0].positions[0] = 0.2;
  posture.points[0].positions[1] = 0.2;  
  posture.points[0].positions[2] = 0.2;
  posture.points[0].positions[3] = 0.2;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void CallBridge::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(4);
  posture.joint_names[0] = "left_finger_1_link";
  posture.joint_names[1] = "left_finger_2_link";
  posture.joint_names[2] = "right_finger_1_link";
  posture.joint_names[3] = "right_finger_2_link";
  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(4);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;  
  posture.points[0].positions[2] = 0;
  posture.points[0].positions[3] = 0;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

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


