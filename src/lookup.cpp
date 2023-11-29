#include <ros/ros.h>
#include <tf/transform_listener.h>

void transformPoint(tf::TransformListener& listener) {
    tf::StampedTransform transform;

    try {
        listener.waitForTransform("lebai_base_link", "lebai_link_6", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("lebai_base_link", "lebai_link_6", ros::Time(0), transform);

        // Process the obtained transform as needed

        // For example, you can get translation and rotation
        tf::Vector3 translation = transform.getOrigin();
        tf::Quaternion rotation = transform.getRotation();

        // Now you can use translation and rotation in your application
        ROS_INFO("Transform Translation: (%f, %f, %f)", translation.x(), translation.y(), translation.z());
        ROS_INFO("Transform Rotation: (%f, %f, %f, %f)", rotation.x(), rotation.y(), rotation.z(), rotation.w());
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Error transforming point: %s", ex.what());
        // Handle the error as needed
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lookup");
    ros::NodeHandle nh;

    tf::TransformListener listener;

    // Call the function to perform the transform
    transformPoint(listener);

    return 0;
}
