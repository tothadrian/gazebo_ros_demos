#include <cstdio>
#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud.h"

//Conversion
#include <sensor_msgs/point_cloud_conversion.h>


/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter(double snap_duration)
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(snap_duration), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void timerCallback(const ros::TimerEvent& timer_ev)
  {

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    AssembleScans2 srv;
    sensor_msgs::PointCloud2 cloud2;
    srv.request.begin = ros::Time(0,0);
    srv.request.end   = timer_ev.current_real;

    // Make the service call
    if (client_.call(srv))
    {
      ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.width)) ;
      //sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud,cloud2);
      pub_.publish(srv.response.cloud);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  double duration;
  //read from parameter server how often the service should be called
  n.param("periodic_snapshotter/duration", duration, 1.0);
  PeriodicSnapshotter snapshotter(duration);
  ros::spin();
  return 0;
}