/*
 * Created By: Gareth Ellis
 * Created On: February 4th, 2017
 * Description:
 *  - A collection of useful functions used across multiple projects
 *  - All functions should be preface with "SB_" to indicate they are
 *  functions "owned" by our team, preventing confusion with other,
 *  similarly named functions in other packages
 */

#ifndef UTILS_UTILS_H
#define UTILS_UTILS_H

#include <ros/ros.h>

/**
 * Get a param
 *
 * Gets a param with a given name and sets a given variable to it, or to a
 * default value if the param could not be retrieved
 * @param nh the nodehandle on which to look for the param
 * @param param_name the name of the param to get
 * @param param_val the variable to set with the retrieved param value
 * @param default_val the value to set param_val to if no param was found
 */
template <typename T>
void SB_getParam(ros::NodeHandle& nh, const std::string& param_name,
                 T& param_val, const T& default_val);

#endif //UTILS_UTILS_H
