//
// Created by sb on 25/03/17.
//

#include <ZedFilter.h>
#include <fstream>
#include <ros/package.h>

// The constructor
ZedFilter::ZedFilter(int argc, char **argv, std::string node_name) :
    tfListener(tfBuffer)
{
    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/zed/camera/point_cloud/cloud_corrected";
    int queue_size = 1;
    raw_image_subscriber = nh.subscribe(camera_image_topic_name, queue_size, &ZedFilter::imageCallBack, this);
    
    // Setup Publisher(s)
    std::string filtered_image_topic_name = "/zed_filter/filtered_point_cloud";
    filtered_image_publisher = nh.advertise<PointCloudRGB>(filtered_image_topic_name, queue_size);

    // Get Parameters
    SB_getParam(private_nh, "base_frame", base_link_name, std::string("base_link"));

    // Start dynamic reconfigure stuff
    boost::recursive_mutex::scoped_lock dyn_reconf_lock(config_mutex);
    dyn_reconf_lock.unlock();
    f = boost::bind(&ZedFilter::dynamicReconfigureCallback, this, _1, _2);
    server.setCallback(f);

    filter = PointCloudFilter(filter_values);
    last_message_time = std::chrono::high_resolution_clock::now();

    // Obtain default filter value parameters
    zed_filter::ZedHSVFilterConfig config;
    server.getConfigDefault(config);
    server.updateConfig(config);
}

void ZedFilter::dynamicReconfigureCallback(zed_filter::ZedHSVFilterConfig &config, uint32_t level){
    ROS_INFO_STREAM("Reconfigure called");
    filter_values.h_min = config.h_min;   
    filter_values.h_max = config.h_max;   
    filter_values.s_min = config.s_min;   
    filter_values.s_max = config.s_max;   
    filter_values.v_min = config.v_min;   
    filter_values.v_max = config.v_max;   
}

void ZedFilter::imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {

    filter = PointCloudFilter(filter_values);

    sensor_msgs::PointCloud2 transformed_input;
    //std::chrono::high_resolution_clock::time_point init_callback = std::chrono::high_resolution_clock::now();
    geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
                base_link_name,
                zed_camera_output->header.frame_id,
                zed_camera_output->header.stamp,
                ros::Duration(1.0)
        );
    pcl::PCLPointCloud2 temp;
    tf::StampedTransform tf_st;
    sensor_msgs::PointCloud2 temp_sensor_msgs_pc2;
    tf::transformStampedMsgToTF(transformStamped, tf_st);
    pcl_ros::transformPointCloud(base_link_name, tf_st, *zed_camera_output, temp_sensor_msgs_pc2);
    pcl_conversions::toPCL(temp_sensor_msgs_pc2, temp);
    PointCloudRGB::Ptr point_cloud_RGB(new PointCloudRGB);
    pcl::fromPCLPointCloud2(temp, *point_cloud_RGB);
    // Filter Values

    PointCloudRGB::Ptr output_cloud(new PointCloudRGB);
    filter.filterCloud(point_cloud_RGB, output_cloud);
    filtered_image_publisher.publish(output_cloud);
    //output_cloud->header.frame_id = "/zed_current_frame";
    // Publish output
    //filtered_image_publisher.publish(output_cloud);
    //std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
    //auto time_to_pub_cloud = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
    //auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(t5 - init_callback).count();

}
