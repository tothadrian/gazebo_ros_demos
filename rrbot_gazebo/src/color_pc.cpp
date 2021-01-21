#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


using namespace pcl;
using namespace std;

class ProjectPointcloud
{
    public:
        cv_bridge::CvImagePtr cv_ptr;

        ProjectPointcloud(string source_frame, string target_frame, string camera_topic){ 
            //boost::shared_ptr<cv_bridge::CvImage>cv_ptr(new cv_bridge::CvImage);
            string info_topic=camera_topic+"/camera_info";
            cam_info_=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic); //rrbot/camera1->belt_camera
            //cout<<info_topic<<endl;
            sub_ = nh_.subscribe<pcl::PCLPointCloud2>("filtered_pc", 1, boost::bind(&ProjectPointcloud::project_pc, this, _1, cam_info_, source_frame, target_frame));
            tfListener_= new tf2_ros::TransformListener(tfBuffer_);
            pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("colored_pc", 1);
        }

        void project_pc(const pcl::PCLPointCloud2ConstPtr& original_cloud, const sensor_msgs::CameraInfoConstPtr& info_msg, 
        string source_frame, string target_frame)
        {
            //Read camera model for projection
            image_geometry::PinholeCameraModel cam_model_;
            cam_model_.fromCameraInfo(info_msg);

            PointCloud<PointXYZ>::Ptr aux_cloud(new PointCloud<PointXYZ>);
            //Convert pcl::pointcloud2 to pointcloud to be able to access individual points
            fromPCLPointCloud2(*original_cloud, *aux_cloud);

            //try to get a transform between the frame of the pointcloud and the camera
            bool success=false;
            geometry_msgs::TransformStamped transformStamped;
            while (!success){
                try{
                    transformStamped = tfBuffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
                    success=true;
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    success=false;
                    ros::Duration(1).sleep();
                    continue;
                }
            }

            PointCloud<PointXYZ>::Ptr transformed_cloud(new PointCloud<PointXYZ>);
            PointCloud<PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);

            //transform cloud to image frame
            pcl_ros::transformPointCloud(*aux_cloud, *transformed_cloud, transformStamped.transform);
            //int cloud_size=aux_cloud->width;

            cv::Point2d projected_point;
            pcl::PointXYZRGB colored_point;
            cv::Vec3b pixel;
            int row, col;

            for (auto const point : transformed_cloud->points)
            {
            // projection_matrix is the matrix you should use if you don't want to use project3dToPixel() and want to use opencv API
            // cv::Matx34d projection_matrix=cam_model_.fullProjectionMatrix();
            //project point to image frame and get the pixel coordinates
                projected_point=cam_model_.project3dToPixel(cv::Point3d(point.x, point.y, point.z) );
                //cout<<projected_point<<endl;
                row=round(projected_point.y);
                col=round(projected_point.x);

                //check for null pointer (this should be done in a more appropriate way) TODO!
                cv::Mat img;
                success=false;
                while (!success){
                    if (cv_ptr){
                        success=true;
                        img=cv_ptr->image;
                    }
                    else{
                        success=false;
                        ros::Duration(1).sleep();
                        continue;
                    }

                }

                //check if there is color information from the camera for the given point
                if (row<img.size().height && col<img.size().width)
                {
                    //get the point coordinates from the transformed cloud
                    colored_point.x=point.x;
                    colored_point.y=point.y;
                    colored_point.z=point.z;
                    //get the color values from the image
                    pixel = img.at<cv::Vec3b>(row,col);
                    colored_point.b=pixel[0];
                    colored_point.g=pixel[1];
                    colored_point.r=pixel[2];
                    //add the point to the new colored cloud
                    colored_cloud->push_back(colored_point);
                }
            }
            sensor_msgs::PointCloud2 colored_pc;
            //convert to sensor_msgsPointcloud2 (this might be unnecessery in the future)
            pcl::toROSMsg(*colored_cloud,colored_pc);
            //write time and frame to the header before publishing
            colored_pc.header.stamp = ros::Time::now();
            colored_pc.header.frame_id = target_frame;
            pub_.publish(colored_pc);
        }
        
        void imageCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            //this should be synchronized to the pointcloud subscriber, but I couldn't make that work
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }

    private:
       ros::NodeHandle nh_; 
       ros::Subscriber sub_;
       boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info_;
       tf2_ros::Buffer tfBuffer_;
       tf2_ros::TransformListener* tfListener_;
       ros::Publisher pub_;
};

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"color_pc",1);
    ros::NodeHandle nh;

    string source_frame, target_frame, camera_topic;
    //Frames for tf transforms
    nh.param<std::string>("color_pc/source_frame", source_frame, "conveyor_reference");
    nh.param<std::string>("color_pc/target_frame", target_frame, "camera_link_optical");
    //where the camera publishes its data
    nh.param<std::string>("color_pc/camera_topic", camera_topic, "rrbot/camera1"); 
    ProjectPointcloud projector(source_frame, target_frame, camera_topic);

    //subscribe to image topic using image_transport for conversion to opencv compatible data
    image_transport::ImageTransport it(nh); 
    string image_topic=camera_topic+"/image_raw"; 
    image_transport::Subscriber sub = it.subscribe(image_topic, 1, &ProjectPointcloud::imageCallback, &projector); //rrbot/camera1->belt_camera
    //cout<<image_topic<<endl;
    ros::spin();
}