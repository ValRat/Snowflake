/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <FinalDecision.h>


FinalDecision::FinalDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string lidar_decision_topic_name = "/lidar_decision/twist";
    std::string vision_decision_topic_name = "/vision_decision/twist";
    std::string gps_decision_topic_name = "/gps_decision/twist";

    int refresh_rate = 10;
    lidar_subscriber = public_nh.subscribe(lidar_decision_topic_name, refresh_rate, &FinalDecision::lidarCallBack, this);
    vision_subscriber = public_nh.subscribe(vision_decision_topic_name, refresh_rate, &FinalDecision::visionCallBack, this);
    gps_subscriber = public_nh.subscribe(gps_decision_topic_name, refresh_rate, &FinalDecision::gpsCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("twist");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}


void FinalDecision::lidarCallBack(const decision::TwistConfidence::ConstPtr& lidar_decision) {
    // Deal with new messages here
    recent_lidar = *lidar_decision;
    arbitrator(recent_lidar,recent_vision,recent_gps);
}


void FinalDecision::visionCallBack(const decision::TwistConfidence::ConstPtr& vision_decision) {
    // Deal with new messages here
    recent_vision = *vision_decision;
    arbitrator(recent_lidar,recent_vision,recent_gps);
}


void FinalDecision::gpsCallBack(const decision::TwistConfidence::ConstPtr& gps_decision) {
    // Deal with new messages here
    recent_gps = *gps_decision;
    arbitrator(recent_lidar,recent_vision,recent_gps);
}

void FinalDecision::imuCallback(const sensor_msgs::Imu::ConstPtr& imu){
    last_imu_callback = imu->header.stamp;
}

void FinalDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}


geometry_msgs::Twist FinalDecision::arbitrator(decision::TwistConfidence recent_lidar, decision::TwistConfidence recent_vision, decision::TwistConfidence recent_gps){
    // TODO: We should probably wait until we have messages from all 3 decision nodes before we make a decision here
    // TODO: Should we? Don't we want to be able to operate with just some of the nodes?
    if(recent_lidar.twist.angular.z != 0)
        publishTwist(recent_lidar.twist);
    else if(fabs(recent_vision.twist.angular.z) != 0)
        publishTwist(recent_vision.twist);
    else {
        /*
        if (ros::Time::now() - last_imu_callback > ros::Duration(0.5)) {
            geometry_msgs::Twist stop; // Technically can assume this will give it 0s, but explicitely written will help;
            stop.angular.x = 0; stop.angular.y = 0; stop.angular.z = 0;
            stop.linear.x = 0; stop.linear.y = 0; stop.linear.z = 0;
            publishTwist(stop);
        } else {
            */
            publishTwist(recent_gps.twist);
            /*
        }
        */
    }
}

bool FinalDecision::turning(geometry_msgs::Twist twist){
    return (twist.angular.z != 0);
}
