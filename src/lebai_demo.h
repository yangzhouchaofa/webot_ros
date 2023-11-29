#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/tf.h>
bool success;

class LeBai
{
    public:
	geometry_msgs::Pose cur_pose;
    LeBai(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &MoveGroup)
    {
        this->n=nh;
        this->manipulator=&MoveGroup;
        manipulator->setGoalPositionTolerance(0.005);//位置误差容忍度，单位：米
        manipulator->setGoalOrientationTolerance(0.01);//姿态误差容忍度，单位：弧度
        manipulator->setGoalJointTolerance(0.005);//关节角度误差容忍度，单位：弧度
        manipulator->setPoseReferenceFrame("base_link");//设置机械臂末端位姿参考坐标系
        end_effector_link = manipulator->getEndEffectorLink(); //获取机械臂终端link的名称
        manipulator->allowReplanning(true);//允许重新规划
		manipulator->setPlanningTime(10.0);//设置最大规划时间
		manipulator->setPlannerId("TRRT");//设置ompl算法
        set_MaxVelocity_Accelerate(0.1,0.1);//设置末端运动最大速度，加速度，前期尽量低速，熟悉后可增大些，不建议超过0.5
        move_Look();//运动到look位置，可在moveit_config/srdf文件中增改
    }
	~LeBai()
	{
		
	}
	 
	geometry_msgs::Pose get_current_pose()//获取当前末端相对base_link位姿
	{
		double roll,pitch,yaw;
		geometry_msgs::Quaternion q1;
		cur_pose=manipulator->getCurrentPose(end_effector_link).pose;
		q1.x=cur_pose.orientation.x;
		q1.y=cur_pose.orientation.y;
		q1.z=cur_pose.orientation.z;
		q1.w=cur_pose.orientation.w;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(q1,quat);
		tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
		printf("roll:%f pitch:%f  yaw:%f   \n",roll,pitch,yaw);
	} 
	
    void set_MaxVelocity_Accelerate(double max_velocity_scaling_factor,double max_acceleration_scaling_factor)//设置最大速度
    {
        manipulator->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
        manipulator->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
    }

    void move_Zero()//移动到预先设置好的零点位置
    {
        manipulator->setNamedTarget("zero"); 
        manipulator->move();
    }

	void move_Look()//移动到预先设置好的look位置
    {
        manipulator->setNamedTarget("look"); 
        manipulator->move();
    }
	
    void base_car_collision() //生成矩形障碍物，可用于避障
	{
		
		// Now let's define a collision object ROS message for the robot to avoid.

		ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    	ros::WallDuration sleep_t(0.5);
    	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    	{
     	 sleep_t.sleep();
    	}
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit_msgs::PlanningScene planning_scene;
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = "base_collision_link";

		// The id of the object is used to identify it.
		collision_object.id = "base_car";

		// Define a box to add to the world.
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 0.5766 ;
		primitive.dimensions[primitive.BOX_Y] =0.70;
		primitive.dimensions[primitive.BOX_Z] = 0.257;

		// Define a pose for the box (specified relative to frame_id)
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0.0;
		box_pose.position.y = 0.0;
		box_pose.position.z = -0.01/2 -0.02;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		planning_scene.world.collision_objects.push_back(collision_object);
    	planning_scene.is_diff = true;
    	planning_scene_diff_publisher.publish(planning_scene);

		ROS_INFO("Added an table into the world");
	}

    void floor_collision() //生成矩形障碍物，可用于避障
	{
		
		// Now let's define a collision object ROS message for the robot to avoid.

		ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    	ros::WallDuration sleep_t(0.5);
    	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    	{
     	 sleep_t.sleep();
    	}
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit_msgs::PlanningScene planning_scene;
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = "floor_link";

		// The id of the object is used to identify it.
		collision_object.id = "floor";

		// Define a box to add to the world.
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 2;
		primitive.dimensions[primitive.BOX_Y] = 2;
		primitive.dimensions[primitive.BOX_Z] = 0.01;

		// Define a pose for the box (specified relative to frame_id)
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0.0;
		box_pose.position.y = 0.0;
		box_pose.position.z = -0.01/2 -0.02;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		planning_scene.world.collision_objects.push_back(collision_object);
    	planning_scene.is_diff = true;
    	planning_scene_diff_publisher.publish(planning_scene);

		ROS_INFO("Added an table into the world");
	}

    void moveJ(double j1,double j2,double j3,double j4,double j5,double j6)//正解，机械臂六个关节角度值
    {
        std::vector<double> joint={j1,j2,j3,j4,j5,j6};
        manipulator->setJointValueTarget(joint);
        manipulator->move();
    }

	void moveP(double x,double y,double z)//逆解，相对base_link的xyz位置，以起始点姿态作为目标点姿态
    {
       geometry_msgs::Pose target_pose;
	   target_pose = manipulator->getCurrentPose(end_effector_link).pose;

	   target_pose.position.x = x;
	   target_pose.position.y = y;
	   target_pose.position.z = z;
		
		manipulator->setStartStateToCurrentState();
		manipulator->setPoseTarget(target_pose);

		
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::planning_interface::MoveItErrorCode success = manipulator->plan(plan);

		ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

		
		if (success) {
			manipulator->execute(plan);
			sleep(1);
		}
      
    }

    void moveP(double x,double y,double z,double rx,double ry,double rz)//逆解，相对base_link的x，y，z以及欧拉角
    {
       geometry_msgs::Pose target_pose;
		target_pose.position.x = x;
		target_pose.position.y = y;
		target_pose.position.z = z;

		
		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(rx/57.29578, ry/57.29578, rz/57.29578);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();

		
		manipulator->setStartStateToCurrentState();
		manipulator->setPoseTarget(target_pose);

		
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::planning_interface::MoveItErrorCode success = manipulator->plan(plan);

		ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

		
		if (success) {
			manipulator->execute(plan);
			sleep(1);
		}
    }

	void moveP(double x,double y,double z,double qx,double qy,double qz,double qw)//逆解，相对base_link的x，y，z以及四元数xyzw
    {
       geometry_msgs::Pose target_pose;
		target_pose.position.x = x;
		target_pose.position.y = y;
		target_pose.position.z = z;
		target_pose.orientation.x = qx;
		target_pose.orientation.y = qy;
		target_pose.orientation.z = qz;
		target_pose.orientation.w = qw;

		
		manipulator->setStartStateToCurrentState();
		manipulator->setPoseTarget(target_pose);

		
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::planning_interface::MoveItErrorCode success = manipulator->plan(plan);

		ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

		
		if (success) {
			manipulator->execute(plan);
			sleep(1);
		}
    }

    void moveCartesian()//笛卡尔路径规划
    {

        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose start_pose = manipulator->getCurrentPose(end_effector_link).pose;
        start_pose.position.x+=0.05;
        waypoints.push_back(start_pose);

        start_pose.position.y+=0.05;
        waypoints.push_back(start_pose);

        start_pose.position.x-=0.05;
        waypoints.push_back(start_pose);

        start_pose.position.y-=0.05;
        waypoints.push_back(start_pose);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = 0.0;
        int maxtries = 100;   //最大尝试规划次数
        int attempts = 0;     //已经尝试规划次数

        while(fraction < 1.0 && attempts < maxtries)
        {
            fraction = manipulator->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
            
            if(attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }
        
        if(fraction == 1)
        {   
            ROS_INFO("Path computed successfully. Moving the arm.");

            // 运动规划数据
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            // 执行
            manipulator->execute(plan);
            sleep(1);
        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }

    }



    private:
    ros::NodeHandle n;
    std::string  group_name;
    moveit::planning_interface::MoveGroupInterface *manipulator;
	std::string end_effector_link; 

};