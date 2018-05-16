//
// Created by min on 14/05/18.
//
/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: A service that enlarges the occupancy grid to fit the goal point if it's outside of the grid
 */

#ifndef PATHFINDING_IGVC_OCCUPANCYGRIDRESIZESERVICE_H
#define PATHFINDING_IGVC_OCCUPANCYGRIDRESIZESERVICE_H

#include "AStar.h"

class OccupancyGridResizeService {
public:
    /**
     * Given an occupancy grid and a cell location, resize the grid if the cell location is not
     * inside the grid (i.e. row/col is < 0 or >= width/height).
     * The grid will also move its origin if the cell location is to the left or below
     * the initial origin of the grid.
     * If the cell location is already inside the grid, it doesn't change the grid
     *
     * @param grid the occupancy grid passed by reference
     * @param goal the cell location that we want to fit inside the grid
     */
    void static resizeOccupancyGridToFitGoal(nav_msgs::OccupancyGrid &grid, AStar::GridPoint goal);
};

#endif //PATHFINDING_IGVC_OCCUPANCYGRIDRESIZESERVICE_H
