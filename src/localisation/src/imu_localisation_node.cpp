/*
 * Created By: Gareth Ellis
 * Created On: February 22, 2016
 * Description: A node that estimates our current position based solely upon an sensor_msgs/Imu message
 */

#include <IMULocalisationNode.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "my_node";

    // Create an instance of your class
    IMULocalisationNode my_class(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}