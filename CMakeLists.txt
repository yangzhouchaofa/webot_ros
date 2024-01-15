cmake_policy(SET CMP0054 NEW)
cmake_minimum_required(VERSION 2.8.3)
# cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(webots_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation tf cv_bridge image_transport lebai_msgs  moveit_core
moveit_ros_planning_interface cv_bridge)

# list(APPEND CMAKE_PREFIX_PATH "/home/ylc/anaconda3/envs/plastic/lib/python3.9/site-packages/torch/share/cmake")
# find_package(Torch REQUIRED)
# include_directories(${TORCH_INCLUDE_DIRS})
# message(STATUS "TORCH_LIBRARIES: ${TORCH_LIBRARIES}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
  add_message_files(
    FILES
    BoolStamped.msg
    Float64Stamped.msg
    Int32Stamped.msg
    Int8Stamped.msg
    RadarTarget.msg
    RecognitionObject.msg
    StringStamped.msg
    lidar32.msg
  )

## Generate services in the 'srv' folder
  add_service_files(
    FILES
    camera_get_focus_info.srv
    camera_get_info.srv
    camera_get_zoom_info.srv
    display_draw_line.srv
    display_draw_oval.srv
    display_draw_pixel.srv
    display_draw_polygon.srv
    display_draw_rectangle.srv
    display_draw_text.srv
    display_get_info.srv
    display_image_copy.srv
    display_image_delete.srv
    display_image_load.srv
    display_image_new.srv
    display_image_paste.srv
    display_image_save.srv
    display_set_font.srv
    field_get_bool.srv
    field_get_color.srv
    field_get_count.srv
    field_get_float.srv
    field_get_int32.srv
    field_get_node.srv
    field_get_rotation.srv
    field_get_string.srv
    field_get_type.srv
    field_get_type_name.srv
    field_get_vec2f.srv
    field_get_vec3f.srv
    field_import_node.srv
    field_import_node_from_string.srv
    field_remove_node.srv
    field_remove.srv
    field_set_bool.srv
    field_set_color.srv
    field_set_float.srv
    field_set_int32.srv
    field_set_rotation.srv
    field_set_string.srv
    field_set_vec2f.srv
    field_set_vec3f.srv
    get_bool.srv
    get_float.srv
    get_int.srv
    get_string.srv
    get_uint64.srv
    gps_decimal_degrees_to_degrees_minutes_seconds.srv
    lidar_get_frequency_info.srv
    lidar_get_info.srv
    lidar_get_layer_point_cloud.srv
    lidar_get_layer_range_image.srv
    motor_set_control_pid.srv
    mouse_get_state.srv
    node_add_force_or_torque.srv
    node_add_force_with_offset.srv
    node_get_center_of_mass.srv
    node_get_contact_point.srv
    node_get_field.srv
    node_get_id.srv
    node_get_number_of_contact_points.srv
    node_get_name.srv
    node_get_orientation.srv
    node_get_parent_node.srv
    node_get_pose.srv
    node_get_position.srv
    node_get_static_balance.srv
    node_get_status.srv
    node_get_type.srv
    node_get_velocity.srv
    node_remove.srv
    node_reset_functions.srv
    node_move_viewpoint.srv
    node_set_visibility.srv
    node_set_velocity.srv
    pen_set_ink_color.srv
    range_finder_get_info.srv
    receiver_get_emitter_direction.srv
    robot_get_device_list.srv
    robot_set_mode.srv
    robot_wait_for_user_input_event.srv
    save_image.srv
    set_bool.srv
    set_float.srv
    set_float_array.srv
    set_int.srv
    set_string.srv
    skin_get_bone_name.srv
    skin_get_bone_orientation.srv
    skin_get_bone_position.srv
    skin_set_bone_orientation.srv
    skin_set_bone_position.srv
    speaker_is_sound_playing.srv
    speaker_speak.srv
    speaker_play_sound.srv
    supervisor_get_from_def.srv
    supervisor_get_from_id.srv
    supervisor_movie_start_recording.srv
    supervisor_set_label.srv
    supervisor_virtual_reality_headset_get_orientation.srv
    supervisor_virtual_reality_headset_get_position.srv
  )

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    lebai_msgs
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES webots_ros
   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime tf cv_bridge OpenCV lebai_msgs moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(yolov5 src/yolov5.cpp)

target_link_libraries(yolov5
	# ${TORCH_LIBRARIES}
  # ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  
)

add_executable(image_yolo src/image_yolo.cpp)

target_link_libraries(image_yolo
	${catkin_LIBRARIES}
)

add_executable(vel src/vel.cpp)

target_link_libraries(vel
	${catkin_LIBRARIES}
)

add_executable(near_car src/near_car.cpp)

target_link_libraries(near_car
	${catkin_LIBRARIES}
)

add_executable(home src/home.cpp)

target_link_libraries(home
	${catkin_LIBRARIES}
)

add_executable(put src/put.cpp)

target_link_libraries(put
	${catkin_LIBRARIES}
)

add_executable(move src/move.cpp)

target_link_libraries(move
	${catkin_LIBRARIES}
)

add_executable(lookup src/lookup.cpp)

target_link_libraries(lookup
	${catkin_LIBRARIES}
)

#instructions for complete_test client

add_executable(complete_test src/complete_test.cpp)

add_dependencies(complete_test webots_ros_generate_messages_cpp)

target_link_libraries(complete_test
	${catkin_LIBRARIES}
)


add_executable(go_to_position src/go_to_position.cpp)

target_link_libraries(go_to_position
	${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS complete_test
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(image src/image.cpp)
# add_dependencies(webots_ros webots_ros_generate_messages_cpp)
target_link_libraries(image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  opencv_highgui
  )

# catkin_install_python(PROGRAMS
#   scripts/yt_speed.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


