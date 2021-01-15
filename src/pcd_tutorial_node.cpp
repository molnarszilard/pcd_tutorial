

#include "sensor_msgs/Imu.h"
#include <pcd_tutorial/pcd_tutorial_nodeConfig.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/publisher.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
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
#include <pcl/PolygonMesh.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <string>
#include <thread>
#include <visualization_msgs/Marker.h>

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
        std::string cloud_topic = "objects";
        g_init = false;
        pub_.advertise(nh_, cloud_topic.c_str(), 1);
        gravity_marker_pub = nh_.advertise<visualization_msgs::Marker>("imu_out", 1);
        normal_marker_pub = nh_.advertise<visualization_msgs::Marker>("normal_out", 1);
        image_transport::ImageTransport it(nh_);
        pub_image = it.advertise("camera/image", 1);
        sub_ = nh_.subscribe("point_cloud_in", 1, &PCD_Processing::cloudCallback, this);
        config_server_.setCallback(boost::bind(&PCD_Processing::dynReconfCallback, this, _1, _2));
        sub_imu = nh_.subscribe("imu_data", 1, &PCD_Processing::imuCallback, this);
        ros::NodeHandle private_nh("~");
        private_nh.param("frame_id", tf_frame, std::string("pico_zense_depth_frame"));
    }
    ~PCD_Processing() {}

    pcl::PointCloud<pcl::PointXYZRGB>
    planefind(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, int j, pcl::ModelCoefficients::Ptr coefficients)
    {
        size_t size_cloud = cloud_cluster->size();
        int big_plane = 0;
        int horizontal_floor = 0;
        int horizontal_ceiling = 0;
        int vertical = 0;
        printf("cluster %d, size: %zu\n", j, size_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud_cluster, *cloud_colored_cluster);
        /*Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (unsigned int i = 0; i < cloud_colored_cluster->size(); i++)
    {
      centroid += cloud_colored_cluster->points[i].getVector3fMap();
    }
    centroid /= cloud_colored_cluster->size();*/
        Eigen::Vector3f normal;
        normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
        if (coefficients->values[3] < 0)
        {
            normal *= -1;
        }
        float angle = acos(gravity.dot(normal));
        if (cloud_colored_cluster->size() > big_plane_size)
            big_plane = 1;
        if (angle > PI - hv_tolerance)
            horizontal_floor = 1;
        if (angle < hv_tolerance)
            horizontal_ceiling = 1;
        if (angle > PI / 2 - hv_tolerance && angle < PI / 2 + hv_tolerance)
            vertical = 1;
        std::uint8_t r = 0, g = 0, b = 0;
        if (big_plane)
        {
            r = 0;
            g = 125;
            b = 0; // dark green
            if (vertical)
            {
                r = 255;
                g = 255;
                b = 0; //yellow
            }
            if (horizontal_floor)
            {
                r = 178;
                g = 102;
                b = 255; // purple
            }
            if (horizontal_ceiling)
            {
                r = 255;
                g = 0;
                b = 153; //magenta
            }
        }
        else
        {
            r = 0;
            g = 255;
            b = 0; // light green
            if (horizontal_floor)
            {
                r = 178;
                g = 102;
                b = 255; // purple
            }
        }
        std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
        for (int i = 0; i < cloud_colored_cluster->size(); i++)
        {
            cloud_colored_cluster->points[i].rgb = *reinterpret_cast<float *>(&rgb);
        }
        return *cloud_colored_cluster;
    }

    void imageFromPCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        // calibration parameters
        // double K[9] = {385.8655956930966, 0.0, 342.3593021849471,
        //                0.0, 387.13463636528166, 233.38372018194542,
        //                0.0, 0.0, 1.0};

        // EEPROM parameters
        double K[9] = {460.585, 0.0, 334.080,
                       0.0, 460.267, 169.807,
                       0.0, 0.0, 1.0};

        double centerX_ = K[2];
        double centerY_ = K[5];
        double focalX_ = K[0];
        double focalY_ = K[4];
        int height_ = 360;
        int width_ = 640;
        int pixel_pos_x, pixel_pos_y;
        float z, u, v;
        cv::Mat cv_image;
        std::vector<cv::Point2d> imagePoints;

        std::cout << "PointCloud has: " << cloud->size() << " data points." << std::endl;
        //cv_image = Mat(height_, width_, CV_32FC1, 0.0); //Scalar(std::numeric_limits<float>::max()));
        cv::Mat output = cv::Mat::zeros(height_, width_, CV_8UC3);
        for (int i = 0; i < cloud->points.size(); i++)
        {
            uint32_t rgb = cloud->points[i].rgb;

            uint8_t r = cloud->points[i].r;
            uint8_t g = cloud->points[i].g;
            uint8_t b = cloud->points[i].b;
            //std::cout<<"rgb="<<(int)r<<", "<<(int)g<<", "<<(int)b<<std::endl;

            z = cloud->points[i].z * 1000.0;
            u = (cloud->points[i].x * 1000.0 * focalX_) / z;
            v = (cloud->points[i].y * 1000.0 * focalY_) / z;

            pixel_pos_x = (int)(u + centerX_);
            pixel_pos_y = (int)(v + centerY_);

            if (pixel_pos_x > (width_ - 1))
            {
                pixel_pos_x = width_ - 1;
            }
            else if (pixel_pos_x < 0)
            {
                pixel_pos_x = -pixel_pos_x;
            }
            if (pixel_pos_y > (height_ - 1))
            {
                pixel_pos_y = height_ - 1;
            }
            else if (pixel_pos_y < 0)
            {
                pixel_pos_y = -pixel_pos_y;
            }

            output.at<cv::Vec3b>(pixel_pos_y, pixel_pos_x)[0] = b;
            output.at<cv::Vec3b>(pixel_pos_y, pixel_pos_x)[1] = g;
            output.at<cv::Vec3b>(pixel_pos_y, pixel_pos_x)[2] = r;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
        ros::Time time = ros::Time::now();
        msg->header.stamp = time;
        msg->header.frame_id = "pico_zense_depth_frame";
        msg->width = output.cols;
        msg->height = output.rows;
        // msg_d->step=depth_image.cols;
        // msg_d->encoding=sensor_msgs::image_encodings::TYPE_16UC1;

        sensor_msgs::CameraInfo camera_info_msg;
        camera_info_msg.width = output.cols;
        camera_info_msg.height = output.rows;
        camera_info_msg.K = {460.58518931365654, 0.0, 334.0805877590529, 0.0, 460.2679961517268, 169.80766383231037, 0.0, 0.0, 1.0};
        camera_info_msg.D = {0.5738898338521625, 1.392306924206032, 0.0016638208693078532, 0.0008464359166083861, -2.049893407299752};
        camera_info_msg.R = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        camera_info_msg.P = {460.58518931365654, 0.0, 334.0805877590529, 0.0, 0.0, 460.2679961517268, 169.80766383231037, 0.0, 0.0, 0.0, 1.0, 0.0};
        camera_info_msg.distortion_model = "plumb_bob";
        camera_info_msg.header.stamp = time;
        camera_info_msg.header.frame_id = "pico_zense_depth_frame";
        pub_image.publish(msg);
        pub_i.publish(camera_info_msg);
    }

    void
    clusterfind(const PointCloud::ConstPtr &cloud_in)
    {
        // Read in the cloud data
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
        sensor_msgs::PointCloud2 cloud_colored_sensor;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
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
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.02);
        int i = 0, nr_points = (int)cloud_filtered->size(), j = 0;

        while (cloud_filtered->size() > 0.1 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);
            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;
            *cloud_colored_pcl = *cloud_colored_pcl + planefind(cloud_plane, j, coefficients);
            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
            j++;
        }

        pcl::toROSMsg(*cloud_colored_pcl, cloud_colored_sensor);
        cloud_colored_sensor.header.frame_id = "pico_zense_depth_frame";
        std::string fields_list = pcl::getFieldsList(cloud_colored_sensor);
        std::cout << "Colored PointCloud before filtering has: " << cloud_colored_sensor.width << " data points."
                  << " points " << fields_list << "\" in frame \"" << cloud_colored_sensor.header.frame_id << std::endl;
        cloud_colored_sensor.header.stamp = ros::Time::now();
        pub_.publish(cloud_colored_sensor);
        imageFromPCD(cloud_colored_pcl);
    }

    void
    dynReconfCallback(pcd_tutorial::pcd_tutorial_nodeConfig &config, uint32_t level)
    {
        leaf_size = config.leafsize;
        th = config.overlap_threshold;
        big_plane_size = config.big_plane_size;
        min_cloud_size = config.min_cloud_size;
        hv_tolerance = config.hv_tolerance * PI / 180;
        lean_tolerance = config.lean_tolerance * PI / 180;
        NormalDistanceWeightP = config.NormalDistanceWeightPlane; //0.1
        MaxIterationsP = config.MaxIterationsPlane;               //100
        DistanceThresholdP0 = config.DistanceThresholdPlane0;     //0.01
        DistanceThresholdP = config.DistanceThresholdPlane;       //0.001
        StddevMulThresh = config.StddevMulThresh;                 //1.0
        MeanK = config.MeanK;                                     //50
    }

    void
    cloudCallback(const PointCloud::ConstPtr &cloud_in)
    {
        clusterfind(cloud_in);
    }

    void
    imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        if (!g_init)
        {
            gravity0[0] = msg->linear_acceleration.x;
            gravity0[1] = msg->linear_acceleration.y;
            gravity0[2] = msg->linear_acceleration.z;
            gravity0.normalize();
            g_init = true;
        }
        gravity[0] = msg->linear_acceleration.x;
        gravity[1] = msg->linear_acceleration.y;
        gravity[2] = msg->linear_acceleration.z;
        gravity.normalize();
        std::cout << "GRAVITY0 X: " << gravity0[0] << ", Y: " << gravity0[1] << ", Z: " << gravity0[2] << std::endl;
        std::cout << "GRAVITY X: " << gravity[0] << ", Y: " << gravity[1] << ", Z: " << gravity[2] << std::endl;
        float g_angle = acos(gravity.dot(gravity0));
        std::cout << "angle of camera: " << g_angle << std::endl;
        if (g_angle > lean_tolerance)
        {
            if (gravity[1] < 0)
                std::cout << "The robot is UPSIDE_DOWN!" << std::endl;
            else
            {
                if (abs(gravity[0]) > abs(gravity[2]))
                {
                    if (gravity[0] > 0)
                        std::cout << "The robot is LEANING RIGHT!" << std::endl;
                    else
                        std::cout << "The robot is LEANING LEFT!" << std::endl;
                }
                else
                {
                    if (gravity[2] > 0)
                        std::cout << "The robot is LEANING FORWARD!" << std::endl;
                    else
                        std::cout << "The robot is LEANING BACKWARDS!" << std::endl;
                }
            }
        }
        visualization_msgs::Marker gravity_marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        gravity_marker.ns = "basic_shapes";
        gravity_marker.id = 1;
        gravity_marker.type = shape;
        gravity_marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        geometry_msgs::Point pt;
        pt.x = centroid(0);
        pt.y = centroid(1);
        pt.z = centroid(2);
        gravity_marker.points.push_back(pt);
        pt.x = centroid(0) + gravity(0);
        pt.y = centroid(1) + gravity(1);
        pt.z = centroid(2) + gravity(2);
        gravity_marker.points.push_back(pt);
        gravity_marker.scale.x = 0.05;
        gravity_marker.scale.y = 0.1;
        gravity_marker.scale.z = 0.05;
        gravity_marker.color.r = 1.0f;
        gravity_marker.color.g = 0.0f;
        gravity_marker.color.b = 0.0f;
        gravity_marker.color.a = 0.5;
        gravity_marker.lifetime = ros::Duration();
        gravity_marker.header.frame_id = tf_frame;
        gravity_marker.header.stamp = ros::Time::now();
        gravity_marker_pub.publish(gravity_marker);
    }

private:
    double PI = 3.14159265;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh;
    std::string tf_frame = "pico_zense_depth_frame";
    float leaf_size = 0.02f;
    float th = 0.6;
    size_t big_plane_size = 1000;
    size_t min_cloud_size = 100;
    ros::Subscriber sub_;
    ros::Subscriber sub_imu;
    ros::Publisher gravity_marker_pub;
    ros::Publisher normal_marker_pub;
    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;

    image_transport::Publisher pub_image;
    ros::Publisher pub_i = nh_.advertise<sensor_msgs::CameraInfo>("camera/info", 1);
    dynamic_reconfigure::Server<pcd_tutorial::pcd_tutorial_nodeConfig> config_server_;
    Eigen::Vector3f gravity, gravity0;
    double hv_tolerance, lean_tolerance;
    bool g_init;
    double NormalDistanceWeightP;
    int MaxIterationsP;
    double DistanceThresholdP;
    double DistanceThresholdP0;
    double NormalDistanceWeightC;
    int MaxIterationsC;
    double DistanceThresholdC;
    double RadiusLimitsMinC;
    double RadiusLimitsMaxC;
    double StddevMulThresh;
    int MeanK;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_tutorial");
    PCD_Processing sf;
    ros::spin();
}