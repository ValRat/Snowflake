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
    SB_getParam(private_nh, "min_height" min_height, 0.25);
    SB_getParam(private_nh, "max_height" max_height, 10);

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
    SB_doTransform(*zed_camera_output, transformed_input, base_link_name);

    // Conversion to PCL datatype
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(*zed_camera_output, temp);
    PointCloudRGB::Ptr point_cloud_RGB(new PointCloudRGB);
    pcl::fromPCLPointCloud2(temp, *point_cloud_RGB);

    // Filter Values
    PointCloudRGB::Ptr output_cloud(new PointCloudRGB);
    pcl::PassThrough<PointRGB> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height, max_height);
    pass.filter(*point_cloud_RGB);
    output_cloud->header.frame_id = base_link_name;

    // Publish output
    filtered_image_publisher.publish(point_cloud_RGB);
}
