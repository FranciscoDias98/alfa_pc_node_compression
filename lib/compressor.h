#include <stdint.h>
#include <ros/ros.h>
// PCL specific includes
#include "octree_pointcloud_compression_2.h"
#include <octree_pointcloud_compression_2.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <inttypes.h>
//#include <pcl/compression/octree_pointcloud_compression.h>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <chrono>
#include <string>
#include <std_msgs/String.h>
#include <bitset>
#include <std_msgs/UInt8MultiArray.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <iterator>
#include <chrono>
#include <time.h>
#include <string>



#include "CompressedPointCloud.h"
#include "alfa_node.h"





using namespace std::chrono;


struct Compression_profile
    {
      double pointResolution;
      double octreeResolution; //const
      bool doVoxelGridDownSampling;
      unsigned int iFrameRate;
      unsigned char colorBitResolution; // const
      bool doColorEncoding;
    };



class Alfa_Pc_Compress: public AlfaNode
{
public:
    Alfa_Pc_Compress();
    void do_Compression(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud);
    void exe_time();
    void set_compression_profile();
    void process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, const sensor_msgs::PointCloud2ConstPtr& header);
    alfa_msg::AlfaConfigure::Response   process_config(alfa_msg::AlfaConfigure::Request &req);

    alfa_msg::AlfaMetrics output_metrics;
    void metrics(std::stringstream& compressed_data, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, double duration);

    void update_compressionSettings(const alfa_msg::AlfaConfigure::Request configs);

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, out_cloud,cluster1,cluster2;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_encoder;
    stringstream compressed_data; // stringstream to store compressed point cloud
    bool show_statistics;
    pcl::io::compression_Profiles_e compression_profile;
    Compression_profile compression_profile_;
    Alfa_Pc_Compress* node;
    compressed_pointcloud_transport::CompressedPointCloud output_compressed;

    //multi
    int number_threads;
    vector<boost::thread *> thread_list;
    boost::mutex mutex_cluster;
    boost::mutex mutex_encoder;
    boost::mutex mutex_compressed_data;
    void run_worker(int thread_number);
    //new
    vector<stringstream> compressed_data_vector;
    vector<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>*> encoder_vector;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_vector;

    bool multi_thread;

};
