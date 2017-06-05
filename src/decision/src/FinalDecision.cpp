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
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string lidar_decision_topic_name = "/lidar_decision/twist";
    std::string vision_decision_topic_name = "/vision_decision/twist";
    std::string gps_decision_topic_name = "/gps_decision/twist";

    int refresh_rate = 10;
    lidar_subscriber = private_nh.subscribe(lidar_decision_topic_name, refresh_rate, &FinalDecision::lidarCallBack, this);
    vision_subscriber = private_nh.subscribe(vision_decision_topic_name, refresh_rate, &FinalDecision::visionCallBack, this);
    gps_subscriber = private_nh.subscribe(gps_decision_topic_name, refresh_rate, &FinalDecision::gpsCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = private_nh.resolveName("twist");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

    // Duration from current time where we consider message as "old", default is 1/2 a second.
    double duration;
    SB_getParam(private_nh, "old_message_threshold", duration, (double) 0.5);
    old_message_threshold = ros::Duration(duration);

    // How much confidence should be decreased by if the message is old
    SB_getParam(private_nh, "confidence_decrease", confidence_decrease, 20.0);

    // Sets up a "clock" which calls the arbitrator function
    SB_getParam(private_nh, "decision_frequency", decision_frequency, 10.0);
    timer = nh.createTimer(ros::Duration(1.0/decision_frequency), &FinalDecision::arbitrator, this);
}


void FinalDecision::lidarCallBack(const decision::TwistConfidence::ConstPtr& lidar_decision) {
    // Deal with new messages here
    recent_lidar = *lidar_decision;
}


void FinalDecision::visionCallBack(const decision::TwistConfidence::ConstPtr& vision_decision) {
    // Deal with new messages here
    recent_vision = *vision_decision;
}


void FinalDecision::gpsCallBack(const decision::TwistConfidence::ConstPtr& gps_decision) {
    // Deal with new messages here
    recent_gps = *gps_decision;
}

void FinalDecision::imuCallback(const sensor_msgs::Imu::ConstPtr& imu){
    last_imu_callback = imu->header.stamp;
}

void FinalDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}


void FinalDecision::arbitrator(const ros::TimerEvent& timerEvent){
    // TODO: We should probably wait until we have messages from all 3 decision nodes before we make a decision here
    // TODO: Should we? Don't we want to be able to operate with just some of the nodes?

    ros::Duration real_callback_period = timerEvent.current_real - timerEvent.last_real;
    if (real_callback_period > ros::Duration(1.0/decision_frequency)) {
        ROS_WARN_STREAM("Arbitrator delay of: " << real_callback_period.nsec << " nsec");
    }

    updateConfidence(recent_lidar);
    updateConfidence(recent_vision);
    updateConfidence(recent_gps);

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

void FinalDecision::updateConfidence(decision::TwistConfidence& twistConfidence) {
    if ((ros::Time::now() - twistConfidence.header.stamp) > old_message_threshold) {
        twistConfidence.confidence = twistConfidence.confidence - confidence_decrease;
    }
}

