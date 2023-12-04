#include <ros/ros.h>
#define HOME system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0.000000,y: -0.500000,z: 0.300000}, orientation: {x: -1.000000, y: 0.000000,z: 0.000000, w: 0.000000}}, common: {vel: 0.100000, acc: 0.100000, time: 0.0, radius: 0.0}}\"")

void wait_for_sleep(int a)
{
        while (a>0)
        {
            sleep(1); 
            ROS_INFO("wait for %d second",a);  
            a--;
        }
}