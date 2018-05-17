//
// Created by min on 05/05/18.
//

#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>
#include <PathFinder.h>

TEST(PathFinder, TestPathWithNoObstacle) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin;

    // set position of the origin
    geometry_msgs::Point position;
    position.x = 3.0;
    position.y = 3.0;
    position.z = 0.0;
    origin.position = position;

    // set orientation of the origin
    tf::Quaternion q;
    tf::Matrix3x3 rotationMatrix = tf::Matrix3x3();
    rotationMatrix.setEulerYPR(0.0, 0.0, 0.0); // only set Z rotation since it's 2D
    rotationMatrix.getRotation(q);
    tf::quaternionTFToMsg(q, origin.orientation);

    /* mapMetaData of OccupancyGrid */
    // initialize mapMetaData
    nav_msgs::MapMetaData mapMetaData;
    mapMetaData.resolution = 2.0;
    mapMetaData.width = 2;
    mapMetaData.height = 2;
    // add origin to mapMetaData
    mapMetaData.origin = origin;

    /* OccupancyGrid */
    // initialize occupancy grid
    nav_msgs::OccupancyGrid grid;
    // set mapMetaData
    grid.info = mapMetaData;
    grid.data = std::vector<int8_t>(4, GRID_FREE);

    geometry_msgs::Point start;
    start.x = 3.0;
    start.y = 3.0;
    geometry_msgs::Point goal;
    goal.x = 5.0;
    goal.y = 5.0;

    nav_msgs::Path path = PathFinder::perform(start, goal, grid);

    for(int i = 0; i < path.poses.size(); i++) {
        std::cout << i << ": (" << path.poses[i].pose.position.x << "," << path.poses[i].pose.position.y << ")" << std::endl;
    }
    EXPECT_EQ(path.poses.size(), 2);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
