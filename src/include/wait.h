#include "ros/ros.h"
// #include "tf/transform_listener.h"
#define HOME system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: -0.1,y: 0.0,z: 0.4}, orientation: {x: 0.9, y: 0.0,z: 0.0, w: 0.4}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"");
#define POSITION2 system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0,y: -0.3,z: 0.4}, orientation: {x: -1, y: 0,z: 0, w: 0}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"")
#define POSITION1 system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0,y: -0.4,z: 0.4}, orientation: {x: -1, y: 0,z: 0, w: 0}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"")
#define POSITION3 system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: 0.2,y: -0.4,z: 0.4}, orientation: {x: -1, y: 0,z: 0, w: 0}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"")
#define POSITION4 system("rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: -0.2,y: -0.4,z: 0.4}, orientation: {x: -1, y: 0,z: 0, w: 0}}, common: {vel: 0.2, acc: 0.1, time: 0.0, radius: 0.0}}\"")


void wait_for_sleep(int a){
    while (a>0){
        sleep(1);
        ROS_INFO("wait for %d second",a);
        a--;
    }
}

// int lebai_move(float x_set,float y_set,float z_set,float a_set ,float b_set,float c_set,float d_set){
//     system("rosservice call /system_service/enable '{}' "); //启动机械臂
//     tf::TransformListener listener_wait;
//     tf::StampedTransform transform_wait;
//     try{
//         listener_wait.waitForTransform("lebai_base_link", "lebai_link_6", ros::Time(0), ros::Duration(3.0));
//         listener_wait.lookupTransform("lebai_base_link", "lebai_link_6", ros::Time(0), transform_wait);
//         ROS_INFO("Transform values: Translation(x=%f, y=%f, z=%f), Rotation(x=%f, y=%f, z=%f, w=%f)",
//                             transform_wait.getOrigin().x(), transform_wait.getOrigin().y(), transform_wait.getOrigin().z(),
//                             transform_wait.getRotation().x(), transform_wait.getRotation().y(), transform_wait.getRotation().z(), transform_wait.getRotation().w());}
//     catch (tf::TransformException& ex){
//         ROS_ERROR("Error transforming point: %s", ex.what());}
//     if (transform_wait.getOrigin().x() < (x_set + 0.1) && transform_wait.getOrigin().x() > (x_set - 0.1) &&
//         transform_wait.getOrigin().y() > (y_set - 0.1) && transform_wait.getOrigin().y() < (y_set + 0.1) &&
//         transform_wait.getOrigin().z() > (z_set - 0.1) && transform_wait.getOrigin().z() < (y_set + 0.1)) {
//         ROS_INFO("Move already");
//     } 
//     else{
//         char str[256];
//         sprintf(str,"rosservice call /move_joint \"{is_joint_pose: 0, cartesian_pose: {position: {x: %f,y: %f,z: %f}, orientation: {x: %f, y: %f,z: %f, w: %f}}, common: {vel: 0.1, acc: 0.1, time: 0.0, radius: 0.0}}\"",x_set,y_set,z_set,a_set,b_set,c_set,d_set);
//         system(str);        
//         wait_for_sleep(12);}
//         return 0;
// }