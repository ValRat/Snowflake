<<<<<<< HEAD
/*
 * Created By: Min Gyo Kim
 * Created On: May 19th 2018
 * Description: Unit Tests for Path Finder
 */

#include <gtest/gtest.h>
#include <PathFinder.h>
#include "PathFinderTestUtils.h"

TEST(PathFinder, ProcessGridAndGetStartAndGoalOnGrid) {
    /* origin of OccupancyGrid */
    // initialize origin of occupancy grid
    geometry_msgs::Pose origin =
            PathFinderTestUtils::constructPose(3.0, 3.0, 0.0);

    /* map_meta_data of OccupancyGrid */
    // initialize map_meta_data
    nav_msgs::MapMetaData map_meta_data;
    map_meta_data.resolution = 2.0;
    map_meta_data.width      = 2;
    map_meta_data.height     = 3;
    // add origin to map_meta_data
    map_meta_data.origin = origin;
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
