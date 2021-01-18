#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

using namespace std;
class PointcloudFilter
{
  public:
    tf2_ros::Buffer tfBuffer;
    PointcloudFilter(string source_frame, string target_frame, double max_dist)
    {
        // Create a ROS publisher for the output point cloud
      pub_ = nh_.advertise<pcl::PCLPointCloud2> ("filtered_pc", 1);

      // Create a ROS subscriber for the input point cloud
      sub_ = nh_.subscribe<pcl::PCLPointCloud2>("assembled_cloud", 1, boost::bind(&PointcloudFilter::process_cloud, this, _1, source_frame, target_frame, max_dist));
    }
    
    void process_cloud (const pcl::PCLPointCloud2ConstPtr& cloud, std::string source_frame, std::string target_frame, double max_dist)
    {
      // ros::Rate rate(10.0);
      bool success=false;
      geometry_msgs::TransformStamped transformStamped;
      while (!success){
        
        try{
          transformStamped = tfBuffer.lookupTransform(source_frame, target_frame, ros::Time(0));
          success=true;
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          success=false;
          ros::Duration(0.1).sleep();
          continue;
        }
      }


      // Create filter
      pcl::PassThrough<pcl::PCLPointCloud2> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (transformStamped.transform.translation.x-max_dist, transformStamped.transform.translation.x);
      pcl::PCLPointCloud2* pass_filter_output=new pcl::PCLPointCloud2;
      pass.filter (*pass_filter_output);

      pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
      pcl::PCLPointCloud2ConstPtr cloudPtr(pass_filter_output);
      voxelGrid.setInputCloud(cloudPtr);
      // set the leaf size (x, y, z)
      voxelGrid.setLeafSize(0.01, 0.01, 0.01);
      // apply the filter to dereferenced cloudVoxel
      pcl::PCLPointCloud2 filtered_pc;
      voxelGrid.filter(filtered_pc);

      // Publish the data
      pub_.publish (filtered_pc);
    }
    private:
      ros::Publisher pub_;
      ros::Subscriber sub_;
      ros::NodeHandle nh_;

};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_filter");
  ros::NodeHandle nh;
  std::cout << "Process_pointcloud node initialised" << std::endl;
  std::string source_frame, target_frame;
  double max_dist;
  nh.param("pcl_filter/max_dist", max_dist, 2.0);
  nh.param<std::string>("pcl_filter/source_frame", source_frame, "base_link");
  nh.param<std::string>("pcl_filter/target_frame", target_frame, "conveyor_reference");
  PointcloudFilter filter(source_frame, target_frame, max_dist);
  tf2_ros::TransformListener tfListener(filter.tfBuffer);
  // Spin
  ros::spin ();
}