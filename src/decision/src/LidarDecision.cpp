/*
 * Created By: Gareth Ellis
 * Created On: January 26, 2016
 * Description: The Lidar decision node, takes in a raw Lidar scan
 *              and broadcasts a recommended Twist message
 */

#include <LidarDecision.h>


// The constructor
LidarDecision::LidarDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    // TODO: We're moving away from the /robot namespace, update it here and in gazebo
    std::string laserscan_topic_name = "/scan";
    uint32_t refresh_rate = 10;
    scan_subscriber = public_nh.subscribe(laserscan_topic_name, refresh_rate,
                                          &LidarDecision::scanCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("twist");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<decision::TwistConfidence>(twist_topic, queue_size);

    // Get Param(s)
    SB_getParam(public_nh, "max_obstacle_angle_diff", max_obstacle_angle_diff, (float) M_PI / 36);
    SB_getParam(public_nh, "max_obstacle_danger_distance", max_obstacle_danger_distance, (float) 2);
    SB_getParam(public_nh, "max_obstacle_danger_angle", max_obstacle_danger_angle, (float) M_PI / 4);
    SB_getParam(public_nh, "twist_angular_speed_multiplier", twist_angular_speed_multiplier, (float) 3);
    SB_getParam(public_nh, "twist_linear_speed_multiplier", twist_linear_speed_multiplier, (float) 3);

    SB_getParam(public_nh, "min_dangerscore", min_dangerscore, (float) 1.7);
    SB_getParam(public_nh, "max_dangerscore", max_dangerscore, (float) 2.4);
    SB_getParam(public_nh, "min_confidence", min_confidence, (float) 60);
    SB_getParam(public_nh, "max_confidence", max_confidence, (float) 90);
}

// This is called whenever a new message is received
    void LidarDecision::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &raw_scan) {
        // Deal with new messages here
        twist_publisher.publish(generate_twist_message(raw_scan));
    }

decision::TwistConfidence LidarDecision::generate_twist_message(const sensor_msgs::LaserScan::ConstPtr &raw_scan) {

    // Find all the obstacles
    std::vector<LidarObstacle> obstacles = findObstacles(*raw_scan, max_obstacle_angle_diff, max_obstacle_distance_diff);

    // Check that we have at least one obstacle
    if (obstacles.size() >= 1){
        // Choose the most dangerous obstacle
        LidarObstacle most_dangerous_obstacle = mostDangerousObstacle(obstacles);

        // Create and return a twist message based on the most dangerous obstacle
        return twist_message_from_obstacle(most_dangerous_obstacle, max_obstacle_danger_distance,
                                           max_obstacle_danger_angle, twist_linear_speed_multiplier, twist_angular_speed_multiplier);
    } else {
        // We don't have any obstacles, return a all zero twist message
        // or go forward very carefully ;D
        decision::TwistConfidence twistConfidence;
        geometry_msgs::Twist forward_with_care;
        forward_with_care.linear.x = 0.3;
        forward_with_care.linear.y = 0;
        forward_with_care.linear.z = 0;
        forward_with_care.angular.x = 0;
        forward_with_care.angular.y = 0;
        forward_with_care.angular.z = 0;
        twistConfidence.twist = forward_with_care;
        twistConfidence.confidence = 20;
        twistConfidence.header.stamp = ros::Time::now();
        return twistConfidence;
    }

}

std::vector<LidarObstacle> LidarDecision::findObstacles(const sensor_msgs::LaserScan &scan,
                                                        float max_obstacle_angle_diff,
                                                        float max_obstacle_distance_diff) {
    // Get the raw scan data
    std::vector<float> scan_data = scan.ranges;

    // Find all ranges (lidar hits) that could be obstacles (initially each laser hit is an obstacle)
    std::vector<LidarObstacle> obstacles = {};
    for (int i = 0; i < scan_data.size(); i++) {
        // Check that obstacle is within valid range
        if (scan_data[i] < scan.range_max && scan_data[i] > scan.range_min) {
            // If so, add it to the obstacles
            obstacles.emplace_back(LidarObstacle(scan.angle_increment * i + scan.angle_min,
                                                 scan_data[i]));
        }
    }

    // Merge together obstacles that could be the same obstacle
    mergeSimilarObstacles(obstacles, max_obstacle_angle_diff, max_obstacle_distance_diff);

    return obstacles;
}

LidarObstacle LidarDecision::mostDangerousObstacle(const std::vector<LidarObstacle> obstacles) {
    // Return obstacle with the greatest danger score
    return *std::max_element(obstacles.begin(), obstacles.end(),
                              [&] (LidarObstacle obs1, LidarObstacle obs2){
                                  return obs1.dangerScore() < obs2.dangerScore();
                              });
}

void LidarDecision::mergeSimilarObstacles(std::vector<LidarObstacle>& obstacles,
                                          float max_angle_diff,
                                          float max_distance_diff) {
    // Ensure the list of obstacles is sorted in order of ascending angle
    std::sort(obstacles.begin(), obstacles.end(),
        [&] (LidarObstacle l1, LidarObstacle l2){
            return l1.getAvgAngle() < l2.getAvgAngle();
        });

    // Merge similar obstacles
    int i = 0;
    while(i < (long)obstacles.size() - 1){
        // Check if angle difference between two consecutive scans is less than max_angle_diff and max_distance_diff
        if (fabs(obstacles[i+1].getMinAngle() - obstacles[i].getMaxAngle()) < max_angle_diff &&
            fabs(obstacles[i+1].getFirstDistance() - obstacles[i].getLastDistance()) < max_distance_diff){
                // Merge next obstacle into current one
                obstacles[i].mergeInLidarObstacle(obstacles[i+1]);
                obstacles.erase(obstacles.begin()+i+1);
        } else {
            i++;
        }
    }
}

decision::TwistConfidence LidarDecision::twist_message_from_obstacle(LidarObstacle obstacle,
                                                                distance_t danger_distance,
                                                                angle_t danger_angle,
                                                                float linear_vel_multiplier,
                                                                float angular_vel_multiplier) {
    decision::TwistConfidence twistConfidence;
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    if (obstacle.getMinDistance() <= danger_distance && abs(obstacle.getAvgAngle()) <= danger_angle) {
        float minDistance = obstacle.getMinDistance();
        // Calculate linear x dependent on how close obstacle is to the robot
        twist.linear.x = sqrt(minDistance);
        // Calculate angular z dependent on how aligned obstacle is to the robot
        float angle_score = fabs(cos(obstacle.getAvgAngle()));
        // Calculate angular z dependent on how close obstacle is to the robot
        float distance_score = 0.4 / minDistance;
        // Choose one score determined if angle or distance is more dangerous
        if (angle_score >= distance_score)
            twist.angular.z = angular_vel_multiplier * angle_score;
        else {
            // If distance score is too large robot will oversteer and come back around, so just set as
            // multiplier value instead (should be set to 1.5). This translates angular z to ~90° turn
            twist.angular.z = (distance_score > 1) ? linear_vel_multiplier :
                              linear_vel_multiplier * distance_score;
        }
        // Check if we should be turning left or right away from obstacle
        if (obstacle.getAvgAngle() > 0)
            twist.angular.z *= -1;

        twistConfidence.header.stamp = ros::Time::now();
        twistConfidence.twist = twist;
        // ---- Conversion
        float slope = 1.0 * (max_confidence - min_confidence)/(max_dangerscore - min_dangerscore);
        float confidence_from_dangerscore = min_confidence + slope * (obstacle.dangerScore() - min_dangerscore);

        // ----
        twistConfidence.confidence = confidence_from_dangerscore;
        return twistConfidence;

    } else {
        twist.linear.x = 0.3;
        twistConfidence.header.stamp = ros::Time::now();
        twistConfidence.twist = twist;
        twistConfidence.confidence = 20;
        return twistConfidence;
    }
}
