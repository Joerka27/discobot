
#include <iostream>
#include <battle_arena/ProjectorInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "projector_generator");

    BattleProjectorInterface bpi;
    ros::Rate r(100);
    while (ros::ok())
    {
      ros::spinOnce();                   // Handle ROS events

      r.sleep();
    }
    return 0;
}
