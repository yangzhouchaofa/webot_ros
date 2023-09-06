import rospy
import signal
from std_msgs.msg import String
from webots_ros.srv import set_float, set_int
from webots_ros.msg import Int32Stamped

TIME_STEP = 32  # 时钟
NMOTORS = 2     # 电机数量
MAX_SPEED = 2.0  # 电机最大速度

speeds = [0.0, 0.0]  # 电机速度值 0～100
motorNames = ["left_motor", "right_motor"]  # 匹配电机名
controllerCount = 0
controllerList = []

def update_speed():
    for i in range(NMOTORS):
        # 更新速度
        set_velocity_client = rospy.ServiceProxy('/robot/' + motorNames[i] + '/set_velocity', set_float)
        set_velocity_srv = set_float.Request()
        set_velocity_srv.value = -speeds[i]
        set_velocity_client.call(set_velocity_srv)

def controller_name_callback(name):
    global controllerCount, controllerList
    controllerCount += 1
    controllerList.append(name.data)
    rospy.loginfo("Controller #%d: %s.", controllerCount, controllerList[-1])

def quit(sig, frame):
    rospy.loginfo("User stopped the '/robot' node.")
    time_step_srv = rospy.ServiceProxy("robot/robot/time_step", set_int)
    time_step_srv(0)
    rospy.signal_shutdown("Quit")
    exit(0)

def keyboard_data_callback(value):
    global speeds
    send = 0
    if value.data == 314:
        speeds = [5.0, -5.0]  # 左转
        send = 1
    elif value.data == 315:
        speeds = [5.0, 5.0]  # 前进
        send = 1
    elif value.data == 316:
        speeds = [-5.0, 5.0]  # 右转
        send = 1
    elif value.data == 317:
        speeds = [-5.0, -5.0]  # 后退
        send = 1
    elif value.data == 32:
        speeds = [0, 0]  # 停止
        send = 1

    if send:
        update_speed()
        send = 0

def main():
    rospy.init_node('robot_init', anonymous=True)
    signal.signal(signal.SIGINT, quit)
    global controllerCount, controllerList

    name_sub = rospy.Subscriber("model_name", String, controller_name_callback)
    while controllerCount == 0 or controllerCount < name_sub.get_num_connections():
        rospy.spin()

    time_step_client = rospy.ServiceProxy("robot/robot/time_step", set_int)
    time_step_srv = set_int.Request()
    time_step_srv.value = TIME_STEP
    time_step_client(time_step_srv)

    if controllerCount == 1:
        controller_name = controllerList[0]
    else:
        wanted_controller = int(input("Choose the # of the controller you want to use:\n"))
        if 1 <= wanted_controller <= controllerCount:
            controller_name = controllerList[wanted_controller - 1]
        else:
            rospy.logerr("Invalid number for controller choice.")
            return

    rospy.loginfo("Using controller: '%s'", controller_name)
    name_sub.unregister()

    # 初始化电机
    for i in range(NMOTORS):
        set_position_client = rospy.ServiceProxy('/robot/' + motorNames[i] + '/set_position', set_float)
        set_position_srv = set_float.Request()
        set_position_srv.value = float("inf")
        if set_position_client(set_position_srv) and set_position_srv.success:
            rospy.loginfo("Position set to INFINITY for motor %s.", motorNames[i])
        else:
            rospy.logerr("Failed to call service set_position on motor %s.", motorNames[i])

        set_velocity_client = rospy.ServiceProxy('/robot/' + motorNames[i] + '/set_velocity', set_float)
        set_velocity_srv = set_float.Request()
        set_velocity_srv.value = 0.0
        if set_velocity_client(set_velocity_srv) and set_velocity_srv.success:
            rospy.loginfo("Velocity set to 0.0 for motor %s.", motorNames[i])
        else:
            rospy.logerr("Failed to call service set_velocity on motor %s.", motorNames[i])

    keyboard_enable_client = rospy.ServiceProxy("/robot/keyboard/enable", set_int)
    keyboard_enable_srv = set_int.Request()
    keyboard_enable_srv.value = TIME_STEP
    if keyboard_enable_client(keyboard_enable_srv) and keyboard_enable_srv.success:
        keyboard_sub = rospy.Subscriber("/robot/keyboard/key", Int32Stamped, keyboard_data_callback)
        while keyboard_sub.get_num_connections() == 0:
            rospy.spin()

        rospy.loginfo("Keyboard enabled.")
        rospy.loginfo("control directions：")
        rospy.loginfo("  ↑  ")
        rospy.loginfo("← ↓ →")
        rospy.loginfo("stop：space")
        rospy.loginfo("Use the arrows in Webots window to move the robot.")
        rospy.loginfo("Press Ctrl+C to stop the node.")
        while not rospy.is_shutdown():
            if not time_step_client(time_step_srv) or not time_step_srv.success:
                rospy.logerr("Failed to call service time_step for next step.")
                break
            rospy.spin()

    else:
        rospy.logerr("Could not enable keyboard, success = %d.", keyboard_enable_srv.success)

    time_step_srv.value = 0
    time_step_client(time_step_srv)
    rospy.shutdown()

if __name__ == '__main__':
    main()
