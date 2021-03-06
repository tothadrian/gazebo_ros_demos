#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "create_conveyor_frame");
  ros::NodeHandle node;

  //read desired frame name from parameter server
  std::string frame_name;
  node.param<std::string>("create_conveyor_frame/frame_name", frame_name, "conveyor_reference");
  double velocity;

  //read desired speed from parameter server
  node.param("create_conveyor_frame/velocity", velocity, 0.1);
  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  ros::Time start_time = ros::Time::now();
  
  //create frame relative to laser (will be handy when filtering)
  transformStamped.header.frame_id = "hokuyo_link";
  transformStamped.child_frame_id = frame_name;
  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
        q.setRPY(0, -1.5708, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(50.0);
  while (node.ok()){
    ros::Duration delta_t = ros::Time::now() - start_time;
    double delta_t_sec = delta_t.toSec();
    //move the frame with constant speed
    transformStamped.transform.translation.z = -velocity*delta_t_sec;
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

};