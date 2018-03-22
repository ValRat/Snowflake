/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: TODO
 */

#ifndef MAPPING_IGVC_OBSTACLEMANAGER_H
#define MAPPING_IGVC_OBSTACLEMANAGER_H

// C++ STD includes
#include <vector>

// TODO: Should we have a general geometry package for these lines? Seems like we use them a lot (in IARRC, etc)...
// TODO: (perhaps these could go in sb_utils
// Snowbots Includes
#include <sb_geom/Spline.h>
#include <sb_geom/Polynomial.h>
#include <sb_geom/PolynomialSegment.h>

// TODO: Should this class be in it's own file? (probably...)
class Cone {
public:
    /**
     * Create a Cone with default x,y position and radius
     */
    Cone() : Cone(0,0,0) {};

    /**
     * Create a cone with a given x,y position and radius
     * @param x
     * @param y
     * @param radius
     */
    Cone(double x, double y, double radius)
    : x(x),
      y(y),
      radius(radius)
    {}

    double x;
    double y;
    double radius;

};

// Implementation of the `==` operator for Cone
inline bool operator==(const Cone& lhs, const Cone& rhs){
    return (
            lhs.x == rhs.x &&
            lhs.y == rhs.y &&
            lhs.radius == rhs.radius
    );
}


class ObstacleManager {
public:

    /**
     * We delete the default constructor to force the user of this class to use
     * one of the other provided constructors
     */
    ObstacleManager() = delete;

    /**
     * Creates a ObstacleManager
     * @param cone_merging_tolerance the minimum distance between the center of
     * two cones for them to be considered different from each other
     * (and so not merged)
     * @param line_merging_tolerance TODO
     */
    explicit ObstacleManager(double cone_merging_tolerance, double line_merging_tolerance);

    /**
     * Add a given cone to our map of the world
     * @param cone the cone to add
     */
    void addObstacle(Cone cone);

    /**
     * Add a given line to our map of the world
     * @param line the line to add
     */
    void addObstacle(sb_geom::PolynomialSegment line);

    /**
     * Get all the cones in our world
     * @return a list of all known cones in our world
     */
    std::vector<Cone> getConeObstacles();

    /**
     * Get all the lines in our world
     * @return a list of all known lines in our world
     */
    std::vector<sb_geom::Spline> getLineObstacle();

private:

    // TODO: Test me!
    /**
     * Finds the minimum distance between a spline line and polynomial line
     *
     * It does this by first sampling points along the spline, finding the closest two to
     * the polynomial, and then re-sampling the section of line between the two closest points
     * until the difference in distance from each of the two points to the line is < `max_err`
     * @param spline a spline line
     * @param poly_line a polynomial line
     * @param num_sample_points the number of sample points to sample from the spline
     * @param max_err the maximum allowable error in the distance
     * @return the minimum distance between the spline and polynomial lines
     */
    double distanceBetweenLines(sb_geom::Spline spline, sb_geom::PolynomialSegment poly_line,
                                unsigned int num_sample_points,
                                double max_err);

    /**
     * Merges the new line into the current line to come up with an updated line
     *
     * Prioritizes new_line over current_line when merging
     *
     * @param current_line our currently known line
     * @param new_line the newly found line
     * @return the merged line
     */
    sb_geom::Spline updateLineWithNewLine(sb_geom::Spline current_line,
                                          sb_geom::PolynomialSegment new_line);

    // the minimum distance between the center of two cones for them to be
    // considered different from each other (and so not merged)
    double cone_merging_tolerance;

    // the minimum distance between two lines for them to be
    // considered different from each other (and so not merged)
    double line_merging_tolerance;

    // all known cones in our world
    std::vector<Cone> cones;

    // all known lines in our world
    std::vector<sb_geom::Spline> lines;
};


#endif //MAPPING_IGVC_OBSTACLEMANAGER_H