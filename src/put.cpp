#include "lebai_demo.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "put"); //初始化ros节点
    ros::NodeHandle n; //Create a node handle //创建节点句柄
    ros::AsyncSpinner spinner(1); //异步启停，开启指定数量的Spinner线程并发执行Callback队列中的可用回调。
    spinner.start();
    system("rosservice call /system_service/enable '{}' "); //启动机械臂
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    LeBai lebai(n,arm);
    lebai.moveJ(-0.3764005358274392, -2.4500549396510976, -0.29279858288767174, -1.8567878699363258, 1.582205308904796, 3.1362237208321933);
    // lebai.moveJ( -1.3957307693774477, -2.078352219986558, -1.3314953238847365, -0.41896850269126573, 1.570412831597925, 3.231426403480346);
    sleep(10);
    system("rosservice call /io_service/set_gripper_position '{val: 100}'");
	ros::shutdown(); 
}

// [ INFO] [1701138603.837467787]: Transform Translation: (0.490528, -0.225778, 0.500268)
// [ INFO] [1701138603.851073822]: Transform Rotation: (0.769227, 0.626288, 0.090561, 0.088608)
