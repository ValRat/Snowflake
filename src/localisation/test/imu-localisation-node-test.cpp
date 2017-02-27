/*
 * Created By: Gareth Ellis
 * Created On: February 25, 2017
 * Description: Tests for IMULocalisationNode
 */

#include <IMULocalisationNode.h>
#include <gtest/gtest.h>

class IMULocalisationNodeTest : public testing::Test{
protected:
    virtual void SetUp(){
    }
};

TEST_F(IMULocalisationNodeTest, calcNewVelocity1D){
    double prev_vel = 10.223;
    double prev_acceleration = 35.22;
    double curr_acceleration = 33.33;
    double delta_time = 0.05;
    ASSERT_DOUBLE_EQ(11.93675, IMULocalisationNode::calcNewVelocity1D(prev_vel, prev_acceleration, curr_acceleration, delta_time));
}

TEST_F(IMULocalisationNodeTest, calcNewVelocity){
    geometry_msgs::Vector3 prev_velocity, prev_acceleration, curr_acceleration;
    prev_velocity.x = 10;
    prev_velocity.y = 20;
    prev_velocity.z = 30; // Flying robot anyone?

    prev_acceleration.x = 1.23;
    prev_acceleration.y = 3.45;
    prev_acceleration.z = 7.39;

    curr_acceleration.x = 0.5;
    curr_acceleration.y = 4.2;
    curr_acceleration.z = 8;

    double delta_time = 0.03;

    geometry_msgs::Vector3 new_velocity = IMULocalisationNode::calcNewVelocity(prev_velocity,
                                                                               prev_acceleration,
                                                                               curr_acceleration,
                                                                               delta_time);
    EXPECT_DOUBLE_EQ(10.02595, new_velocity.x);
    EXPECT_DOUBLE_EQ(20.11475, new_velocity.y);
    EXPECT_DOUBLE_EQ(30.23085, new_velocity.z);
}

TEST_F(IMULocalisationNodeTest, calcPositionChange1D){
    double curr_vel = 10.223;
    double prev_vel = 50.123;
    double delta_time = 0.05;
    ASSERT_DOUBLE_EQ(1.50865, IMULocalisationNode::calcPositionChange1D(prev_vel, curr_vel, delta_time));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}