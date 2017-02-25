/*
 * Created By: Gareth Ellis
 * Created On: February 22, 2016
 * Description: A node that estimates our current position based solely upon an sensor_msgs/Imu message
 */

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

#include <iostream>
#include <sb_utils.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

class IMULocalisationNode {
public:
    IMULocalisationNode(int argc, char **argv, std::string node_name);
    /**
     * Calculates an updated velocity
     *
     * @param prev_velocity our previous velocity
     * @param curr_acceleration our current acceleration
     * @param prev_acceleration our previous acceleration
     *
     * @return our updated velocity
     */
    static geometry_msgs::Vector3 calcNewVelocity(geometry_msgs::Vector3 prev_velocity,
                                                  geometry_msgs::Vector3 prev_acceleration,
                                                  geometry_msgs::Vector3 curr_acceleration,
                                                  double delta_time);
    /**
     * Calculates an updated velocity in 1 dimension
     *
     * @param prev_velocity our previous velocity in the dimension of interest
     * @param curr_acceleration our current acceleration in the dimension of interest
     * @param prev_acceleration our previous acceleration in the dimension of interest
     *
     * @return our updated velocity in the dimension of interest
     */
    static double calcNewVelocity1D(double prev_velocity,
                                    double prev_acceleration,
                                    double curr_acceleration,
                                    double delta_time);
    /**
     * Calculates our new position
     *
     * @param curr_position our previously estimated position
     * @param prev_velocity our previous velocity
     * @param curr_velocity our current velocity
     * @param delta_time the time since we received our last IMU message
     *
     * @return an estimate of our new position
     */
    static geometry_msgs::Point calcNewPosition(geometry_msgs::Point prev_position,
                                                geometry_msgs::Vector3 prev_velocity,
                                                geometry_msgs::Vector3 curr_velocity,
                                                double delta_time);
    /**
     * Calculates the change in position in a single dimension
     *
     * @param acceleration the acceleration in the direction we're updating position for
     * @param velocity the velocity in the direction we're updating position for
     * @param delta_time the time since we last updated the position
     * @param angular_component the value to multiply acceleration and velocity by to get their
     *                          value in the dimension we're concerned about, (ie. sin(theta) or cos(theta))
     *
     * @return the updated position
     */
    static double calcPositionChange1D(double prev_velocity, double curr_velocity, double delta_time);
private:
    /**
     * Callback function for when a new IMU message is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg);

    /**
     * Updates our current position based on the received IMU message
     *
     * @param imu_msg an imu message
     */
    void updateCurrPosition(sensor_msgs::Imu imu_msg);

    /**
     * Publishes an estimate of our current position, based on IMU data received so far
     */
    void publishCurrentPosition();

    bool received_initial_reading; // Whether or not we've received our initial reading
    geometry_msgs::Quaternion initial_orientation; // Our initial orientation
    geometry_msgs::Point curr_position; // Our estimated position
    double prev_imu_msg_time; // The last time we received an IMU message
    geometry_msgs::Vector3 prev_velocity; // The velocity from the previously received IMU message
    geometry_msgs::Vector3 prev_acceleration; // The acceleration from the previously received IMU message

    ros::Subscriber imu_subscriber;
    ros::Publisher location_publisher;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
