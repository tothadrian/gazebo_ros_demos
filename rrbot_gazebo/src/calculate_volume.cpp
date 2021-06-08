#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


using namespace pcl;
using namespace std;

ros::NodeHandle* nhPtr; 
std::vector<ros::Publisher> pub_vec;


    void segment_objects(const PCLPointCloud2ConstPtr& input_cloud)
    {
        PointCloud<PointXYZRGB> *xyzInput= new PointCloud<PointXYZRGB>;
        PointCloud<PointXYZRGB>::Ptr xyzInputPtr (xyzInput);
        fromPCLPointCloud2(*input_cloud, *xyzInput);
        // Create the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud (xyzInputPtr);

        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        // specify euclidean cluster parameters
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (xyzInputPtr);
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract (cluster_indices);
        cout<<"number of clusters"+cluster_indices.size()<<endl;

        pcl::PCLPointCloud2 outputPCL;
        //Create a publisher for each cluster
        for (int i = 0; i < cluster_indices.size(); ++i)
        {
            std::string topicName = "/pcl_cluster" + boost::lexical_cast<std::string>(i);
            ros::Publisher pub = nhPtr->advertise<sensor_msgs::PointCloud2> (topicName, 1);
            pub_vec.push_back(pub);
        }

        int j=0;
        // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
           // create a pcl object to hold the extracted cluster
            pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

            // now we are in a vector of indices pertaining to a single cluster.
            // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {
                xyzInputPtr->points[*pit].r=255;
                xyzInputPtr->points[*pit].g=0;
                xyzInputPtr->points[*pit].b=0;
                clusterPtr->points.push_back(xyzInputPtr->points[*pit]);
            } 
            
            // convert to pcl::PCLPointCloud2
            pcl::toPCLPointCloud2( *clusterPtr,outputPCL);
            pub_vec[j].publish (outputPCL);
            ++j;
                }
    }

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"calculate_volume",1);
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe<PCLPointCloud2>("colored_pc", 1, segment_objects);
    nhPtr = & nh; 
    ros::spin();
}


