#include <pcd_tutorial/voxel_filter_nodeConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/publisher.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <thread>

using namespace pcd_tutorial;
class PCD_Processing
{
public:
    typedef pcl::PointXYZRGB Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef pcl::PointXYZ PointT;

    PCD_Processing()
        : private_nh("~")
    {
        pub_.advertise(nh_, "voxel_out", 1);
        sub_ = nh_.subscribe("point_cloud_in", 1, &PCD_Processing::cloudCallback, this);
        config_server_.setCallback(boost::bind(&PCD_Processing::dynReconfCallback, this, _1, _2));
        ros::NodeHandle private_nh("~");
        private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
    }
    ~PCD_Processing() {}

    void
    cloudCallback(const PointCloud::ConstPtr &cloud_in)
    {
        // Read in the cloud data
        sensor_msgs::PointCloud2 cloud_sensor;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_in, *cloud);
        std::cout << std::endl;
        std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_filtered_voxel);
        std::cout << "PointCloud after filtering has: " << cloud_filtered_voxel->size() << " data points." << std::endl; //*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered_voxel);
        sor.setMeanK(MeanK);
        sor.setStddevMulThresh(StddevMulThresh);
        sor.setNegative(false);
        sor.filter(*cloud_filtered);
        // Create the segmentation object for the planar model and set all the parameters
        

        pcl::toROSMsg(*cloud_filtered, cloud_sensor);
        cloud_sensor.header.frame_id = "pico_zense_depth_frame";
        std::string fields_list = pcl::getFieldsList(cloud_sensor);
        std::cout << "Colored PointCloud before filtering has: " << cloud_sensor.width << " data points."
                  << " points " << fields_list << "\" in frame \"" << cloud_sensor.header.frame_id << std::endl;
        cloud_sensor.header.stamp = ros::Time::now();
        pub_.publish(cloud_sensor);
    }

    void
    dynReconfCallback(pcd_tutorial::voxel_filter_nodeConfig &config, uint32_t level)
    {
        //leaf_size = config.leafsize;
        StddevMulThresh = config.StddevMulThresh;                 //1.0
        MeanK = config.MeanK;                                     //50
    }


private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh;
    std::string tf_frame = "pico_zense_depth_frame";
    float leaf_size = 0.01f;
    ros::Subscriber sub_;
    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;
    dynamic_reconfigure::Server<pcd_tutorial::voxel_filter_nodeConfig> config_server_;
    double StddevMulThresh;
    int MeanK;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_tutorial");
    PCD_Processing sf;
    ros::spin();
}
