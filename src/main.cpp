#include "call_bridge/call_bridge.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "call_bridge");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    const std::string PLANNING_GROUP = "panda_arm";
    //MoveGroup类只需使用要控制和规划的规划组的名称即可轻松设置。
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    CallBridge callBridge(nh, move_group);
    ros::spin();
    return 0;
}