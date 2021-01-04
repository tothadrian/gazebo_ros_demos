#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

ros::Publisher pub;

void process_cloud (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  // Create container variables
  pcl::PCLPointCloud2 filtered_pc;

  // Create filter
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (filtered_pc);

  // Publish the data
  pub.publish (filtered_pc);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ros_pcl_minimal_pub_sub");
  ros::NodeHandle nh;
  std::cout << "Process_pointcloud node initialised" << std::endl;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("rrbot/laser/scan", 1, process_cloud);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("filtered_pc", 1);

  // Spin
  ros::spin ();
}