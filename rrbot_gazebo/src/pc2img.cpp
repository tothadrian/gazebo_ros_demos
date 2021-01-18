//Next step: transform pc to camera frame

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace pcl;

void project_pc(const pcl::PCLPointCloud2ConstPtr& original_cloud, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(info_msg);
    PointCloud<PointXYZ>::Ptr aux_cloud(new PointCloud<PointXYZ>);
    fromPCLPointCloud2(*original_cloud, *aux_cloud);

    int width=aux_cloud->width;
    for (auto const point : aux_cloud->points)
    {

    // projection_matrix is the matrix you should use if you don't want to use project3dToPixel() and want to use opencv API
    // cv::Matx34d projection_matrix=cam_model_.fullProjectionMatrix();
        std::cout<<cam_model_.project3dToPixel(cv::Point3d(point.x,point.y, point.z) )<<std::endl;
    }
}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"pc2img",1);
    ros::NodeHandle nh;

    boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info;
    cam_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/rrbot/camera1/camera_info");

    ros::Subscriber sub = nh.subscribe<pcl::PCLPointCloud2>("filtered_pc", 1, boost::bind(project_pc, _1, cam_info));
    ros::spin();
}