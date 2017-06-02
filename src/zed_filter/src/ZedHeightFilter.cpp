//
// Created by sb on 25/03/17.
//

#include <ZedHeightFilter.h>

// The constructor
ZedHeightFilter::ZedHeightFilter(int argc, char **argv, std::string node_name)
{
    ros::init(argc, argv, node_name);

    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    SB_getParam(private_nh, "min_height", min_height, 0.25);
    SB_getParam(private_nh, "max_height", max_height, 10.0);

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/zed/point_cloud/cloud_registered";
    int queue_size = 1;
    raw_image_subscriber = nh.subscribe(camera_image_topic_name, queue_size, &ZedHeightFilter::imageCallBack, this);
    
    // Setup Publisher(s)
    std::string filtered_image_topic_name = "/zed_height_filter/filtered_point_cloud";
    filtered_image_publisher = nh.advertise<sensor_msgs::PointCloud2>(filtered_image_topic_name, queue_size);

    base_link_name = "base_link";
}

void ZedHeightFilter::imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {
    sensor_msgs::PointCloud2 transformed_input;
    sensor_msgs::PointCloud2 zed_modifiable = *zed_camera_output;
    zed_modifiable.header.frame_id = "zed_actual_frame"; //TODO: Function to just strip off the slash
    SB_doTransform(zed_modifiable, transformed_input, base_link_name);

    // Conversion to PCL datatype
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(transformed_input, temp);
    PointCloudRGB::Ptr point_cloud_RGB(new PointCloudRGB);
    pcl::fromPCLPointCloud2(temp, *point_cloud_RGB);

    // Filter Values
    pcl::PassThrough<PointRGB> pass;
    pass.setInputCloud(point_cloud_RGB);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*point_cloud_RGB);
    //point_cloud_RGB->header.frame_id = base_link_name;
    ROS_INFO_STREAM("OUTPUT CLOUD POINTS: " << point_cloud_RGB->points.size());

    // Publish output
    filtered_image_publisher.publish(point_cloud_RGB);
}
