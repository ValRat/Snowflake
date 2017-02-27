/*
 * Created By: Gareth Ellis
 * Created On: February 4th, 2017
 * Description:
 *  - A collection of useful functions used across multiple projects
 *  - All functions should be preface with "SB_" to indicate they are
 *  functions "owned" by our team, preventing confusion with other,
 *  similarly named functions in other packages
 */

#include <sb_utils.h>

void sb_vector_utils::changeVectorOrientation(geometry_msgs::Vector3& vec,
                                              geometry_msgs::Quaternion& from_orientation,
                                              geometry_msgs::Quaternion& to_orientation) {
    // Convert the orientations to tf::Quaternion
    tf::Quaternion tf_from_orientation, tf_to_orientation;
    tf::quaternionMsgToTF(from_orientation, tf_from_orientation);
    tf::quaternionMsgToTF(to_orientation, tf_to_orientation);
    
    // Convert the vector to tf::Vector3
    tf::Vector3 tf_vec;
    tf::vector3MsgToTF(vec, tf_vec);

    // Calculate the difference between the two orientations
    tf::Quaternion tf_orientation_diff = tf_to_orientation.inverse() * tf_from_orientation;

    // Translate the vector by the difference between the two orientation
    tf::quatRotate(tf_orientation_diff, tf_vec);

    // Convert our vector back to a geometry_msgs::Vector3
    tf::vector3TFToMsg(tf_vec, vec);
}
