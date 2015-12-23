#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "r3map_mapping/CloudArray.h"


class Mapping
{

private:

public:

    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    bool reset_toggle;
    std_msgs::Header header;
    sensor_msgs::PointCloud2 master_map;

    Mapping();
    ~Mapping();
    void velodyne_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void enu_pointcloud_cb(const r3map_mapping::CloudArray::ConstPtr& msg);
    void filter_pc(const sensor_msgs::PointCloud2::ConstPtr& msg);

    ros::Publisher reset = nh.advertise<std_msgs::Empty>("/reset_time", 100);
    ros::Publisher raw_enu_pub = nh.advertise<r3map_mapping::CloudArray>("/raw_enu_points", 100);
    ros::Publisher raw_unfiltered_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/raw_unfiltered_map", 100);
    ros::Publisher master_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    ros::Subscriber raw_velondyne_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, &Mapping::velodyne_cb, this);
    ros::Subscriber raw_enu_sub = nh.subscribe<r3map_mapping::CloudArray>("/raw_enu_points", 100, &Mapping::enu_pointcloud_cb, this);
    ros::Subscriber raw_unfiltered_map_sub = nh.subscribe<sensor_msgs::PointCloud2>("/raw_unfiltered_map", 100, &Mapping::filter_pc, this);

};

Mapping::Mapping(){
    reset_toggle = false;
}

Mapping::~Mapping(){

}

void Mapping::velodyne_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    // On the first iteration the tf should be reset
    // If it is not reset, reset it - It is reset by publishing an Empty() message to the topic /reset_time
    if (reset_toggle == false)
    {
        reset.publish(std_msgs::Empty());
        reset_toggle = true;
    }

    // Cloud to be transformed in the /enu frame
    sensor_msgs::PointCloud2 cloud_out;

    // Get transform from /velodyne to /enu
    bool ready = tf_listener.waitForTransform(msg->header.frame_id, "/enu", msg->header.stamp, ros::Duration(1));

    // If the transfrom was succesful
    if (ready == true)
    {
        // Transform the entire cloud to /enu
        pcl_ros::transformPointCloud("/enu", *msg,  cloud_out, tf_listener);
        ROS_DEBUG_STREAM("Converted to /enu");
    }else{
        ROS_DEBUG_STREAM("Did not convert to /enu. Transform Error");
    }

    // Package the master cloud and the new cloud to be sent to be combined and filtered
    r3map_mapping::CloudArray clouds;
    clouds.cloud.push_back(master_map);
    clouds.cloud.push_back(cloud_out);

    // publish to the raw_enu topic which calls the next function
    raw_enu_pub.publish(clouds);

    ROS_DEBUG_STREAM("Let's party");
    ros::spinOnce();

}

void Mapping::enu_pointcloud_cb(const r3map_mapping::CloudArray::ConstPtr& msg)
{

    // Create a cloud to be the result of cloud1 + master_map
    sensor_msgs::PointCloud2 unfiltered_map;
    // Add the master cloud and the newest cloud and put into unfiltered_map
    pcl::concatenatePointCloud(msg->cloud[0], msg->cloud[1], unfiltered_map);   

    // publish the newest unfiltered map which calls the filtering function
    raw_unfiltered_map_pub.publish(unfiltered_map);

    ROS_DEBUG_STREAM("Added to map");
    ros::spinOnce();

}

void Mapping::filter_pc(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    // Temporary variables to hold the clouds
    pcl::PCLPointCloud2::Ptr unfiltered_map (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr filtered_map (new pcl::PCLPointCloud2 ());
    sensor_msgs::PointCloud2 filtered_map_ros;

    // Convert the unfiltered map into a PCLPointCloud2 data type - put in unfiltered_map variable
    pcl_conversions::toPCL(*msg, *unfiltered_map);

    // Filter the new map
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (unfiltered_map);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*filtered_map);

    // Convert the filtered map back into a ROS PointCloud2 data type
    pcl_conversions::fromPCL(*filtered_map, filtered_map_ros);

    // Set the master map as the new map for the next iteration
    master_map = filtered_map_ros;

    // Publish the master map for viewing
    master_map_pub.publish(master_map);
    ROS_DEBUG_STREAM("Filtered");
    ros::spinOnce();

}

// -------------------------------------- END OF CLASS ----------------------------------------------

int main(int argc, char** argv)
{
    // Initialize node
    ros::init(argc, argv, "Mapping");
    ros::NodeHandle nh;

    // By default, set the message update rate at 5hz
    ros::Rate update_rate(5); 

    // Create a map object
    Mapping map;
    
    // Set Log Level to Debug
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }


    // While the node is running
    while(ros::ok())
    {
        // Refresh the messages at at least 5hz
        update_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

