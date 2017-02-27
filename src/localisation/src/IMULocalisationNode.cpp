/*
 * Created By: Gareth Ellis
 * Created On: February 22, 2016
 * Description: A node that estimates our current position based solely upon an sensor_msgs/Imu message
 */

#include <IMULocalisationNode.h>

IMULocalisationNode::IMULocalisationNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "imu";
    int refresh_rate = 10;
    imu_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &IMULocalisationNode::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = public_nh.resolveName("curr_location");
    uint32_t queue_size = 1;
    location_publisher = public_nh.advertise<geometry_msgs::Point>(topic, queue_size);

    received_initial_reading = false;
    ROS_WARN("Waiting on initial reading");
}

void IMULocalisationNode::subscriberCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // If we've not received a message before, set our initial orientation
    if (!received_initial_reading){
        // Save our intial orientation for later use
        received_initial_reading = true;
        initial_orientation = imu_msg->orientation;
        ROS_INFO("Initial reading received, good to go!");
    }
    updateCurrPosition(*imu_msg);
    publishCurrentPosition();
}

void IMULocalisationNode::updateCurrPosition(sensor_msgs::Imu imu_msg) {

    double delta_time = imu_msg.header.stamp.sec - prev_imu_msg_time;

    geometry_msgs::Vector3 curr_acceleration = imu_msg.linear_acceleration;
    // Translate the acceleration from the current orientation of the IMU to our initial orientation
    sb_vector_utils::changeVectorOrientation(curr_acceleration, imu_msg.orientation, initial_orientation);

    // Update our state
    geometry_msgs::Vector3 curr_velocity = calcNewVelocity(prev_velocity, curr_acceleration,
                                                           prev_acceleration, delta_time);
    curr_position = calcNewPosition(curr_position, prev_velocity, curr_velocity, delta_time);
    prev_acceleration = curr_acceleration;
    prev_velocity = curr_velocity;
    prev_imu_msg_time = imu_msg.header.stamp.sec;
}

geometry_msgs::Vector3 IMULocalisationNode::calcNewVelocity(geometry_msgs::Vector3 prev_velocity,
                                                            geometry_msgs::Vector3 prev_acceleration,
                                                            geometry_msgs::Vector3 curr_acceleration,
                                                            double delta_time) {
    geometry_msgs::Vector3 new_velocity;
    new_velocity.x = calcNewVelocity1D(prev_velocity.x, prev_acceleration.x, curr_acceleration.x, delta_time);
    new_velocity.y = calcNewVelocity1D(prev_velocity.y, prev_acceleration.y, curr_acceleration.y, delta_time);
    // We don't NEED the z component... but 'cuz I can...
    new_velocity.z = calcNewVelocity1D(prev_velocity.z, prev_acceleration.z, curr_acceleration.z, delta_time);
    return new_velocity;
}

double IMULocalisationNode::calcNewVelocity1D(double prev_velocity,
                                              double prev_acceleration,
                                              double curr_acceleration,
                                              double delta_time) {
    return prev_velocity + ((prev_acceleration + curr_acceleration)/2) * delta_time;
}


geometry_msgs::Point IMULocalisationNode::calcNewPosition(geometry_msgs::Point prev_position,
                                                          geometry_msgs::Vector3 prev_velocity,
                                                          geometry_msgs::Vector3 curr_velocity,
                                                          double delta_time) {
    geometry_msgs::Point new_position;
    new_position.x = prev_position.x + calcPositionChange1D(prev_velocity.x, curr_velocity.x, delta_time);
    new_position.y = prev_position.y + calcPositionChange1D(prev_velocity.y, curr_velocity.y, delta_time);
    return new_position;
}

double IMULocalisationNode::calcPositionChange1D(double prev_velocity, double curr_velocity, double delta_time) {
    return 0.5 * (prev_velocity + curr_velocity) * delta_time;
}

void IMULocalisationNode::publishCurrentPosition() {
    location_publisher.publish(curr_position);
}
