#include <stdint.h>
#include <ros/ros.h>
// PCL specific includes
#include "octree_pointcloud_compression_2.h"
//#include <octree_pointcloud_compression_2.h>

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
    void process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, const sensor_msgs::PointCloud2ConstPtr& header);
    alfa_msg::AlfaConfigure::Response   process_config(alfa_msg::AlfaConfigure::Request &req);

    alfa_msg::AlfaMetrics output_metrics;
    void metrics(std::stringstream& compressed_data, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, double duration);

    void update_compressionSettings(const alfa_msg::AlfaConfigure::Request configs);
    void store_occupancy_code_hardware(std::vector<char> vec ,u64 *pointer);

    //test multithread
    void do_compression();
    void my_write_frame_header(std::ostream& compressed_tree_data_out_arg);
    void print_statistics();

    //octants
    void divide_in_octants(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud); // faster, checks one point at time
    void divide_in_octants_2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud); // using PCL function, mid axis values, slower
    void divide_in_octants_multi(int thread_number,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void divide_in_octants_test(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

    void compress_octant(pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_encoder,pcl::PointCloud<pcl::PointXYZRGB>::Ptr octant);
    void print_statistics_octant(pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_encoder); // print compression statistics
    void set_compression_profiles_octants();
    void compress_octant_0();
    void compress_octant_1();
    void compress_octant_2();
    void compress_octant_3();
    void compress_octant_4();
    void compress_octant_5();
    void compress_octant_6();
    void compress_octant_7();

    //test my_octree functions
    //void my_addPointsFromInputCloud();
    //void my_addPointIdx(const int point_idx_arg)

    // Hw functions
    void my_write_frame_header_hw(std::ostream& compressed_tree_data_out_arg);



private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, out_cloud,cluster0,cluster1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr octree_cloud;


    //pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_encoder;
    stringstream compressed_data; // stringstream to store compressed point cloud
    bool show_statistics;
    pcl::io::compression_Profiles_e compression_profile;
    Compression_profile compression_profile_;
    Alfa_Pc_Compress* node;
    compressed_pointcloud_transport::CompressedPointCloud output_compressed,output_compressed1;

    //multi
    int number_threads;
    vector<boost::thread *> thread_list;
    boost::mutex mutex_cluster;
    boost::mutex mutex_encoder;
    boost::mutex mutex_compressed_data1,mutex_compressed_data2;
    //sensor_msgs::PointCloud2ConstPtr& header;
    void run_worker(int thread_number);  // <------------------  APAGAR DEPOIS!!!!!!
    void run_worker_thread(int thread_number,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    //new
    vector<stringstream> compressed_data_vector;
    vector<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>*> encoder_vector;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_vector;

    bool multi_thread,test;
    const char* frame_header_identifier ;

    ///////
    uint64_t point_count_0,point_count_1,point_count_2,point_count_3,point_count_4,point_count_5,point_count_6,point_count_7;
    bool cloud_with_color_;

    //multi thread odd/even frames
    void run_worker_thread_odd_even(int thread_number,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

    //multi thread octants
    void run_worker_thread_octants(uint8_t thread_nummber,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void run_worker_thread_octants_full(uint8_t thread_nummber,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr octant_0,octant_1,octant_2,octant_3,octant_4,octant_5,octant_6,octant_7;
    boost::mutex mux_oct_0,mux_oct_1,mux_oct_2,mux_oct_3,mux_oct_4,mux_oct_5,mux_oct_6,mux_oct_7;

    float mid_x,mid_y,mid_z;


    //pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>::Ptr test_compressor;
    //hw
    u64 *ddr_pointer;
    u_int32_t *hw32_vptr;

    int size_test;
    int depth_test;
    bool hw;


};
