#include "compressor.h"
#include "CompressedPointCloud.h"
#include <chrono>
#include <time.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <string>

// testes tempos
unsigned int x = 0;
unsigned long tempos = 0;
float size =0;
float size2 =0;

pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;

//ros::Publisher pub2;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

Alfa_Pc_Compress::Alfa_Pc_Compress()
{
    std::cout << "entrei no construtor" << std::endl;

    in_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    out_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    set_compression_profile(); // define compression profile
    spin();
}

void Alfa_Pc_Compress::set_compression_profile()
{
    std::cout << "Setting Compression Profile" << std::endl;


    compression_profile = pcl::io::MANUAL_CONFIGURATION;

    show_statistics = true;
    compression_profile_.pointResolution = 0.01;
    compression_profile_.octreeResolution = 0.05; //-----> ALTERAR NESTE!!!!!!!!! voxel size in cubic meters (1m is 0.01 cm)
    compression_profile_.doVoxelGridDownSampling = true;
    compression_profile_.iFrameRate = 10; // number of prediction frames
    compression_profile_.colorBitResolution = 0;
    compression_profile_.doColorEncoding = false;

    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
     //point_cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    //PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile, show_statistics);
}

void Alfa_Pc_Compress::process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, const sensor_msgs::PointCloud2ConstPtr& header)
{
    //do_Compression(output_cloud);
    std::stringstream compressed_data;

    ROS_INFO("Compressing cloud with frame [%s]", output_cloud->header.frame_id.c_str());
    std::cout << output_cloud->header << std::endl;
    //point_cloud_encoder->encodePointCloud(output_cloud,compressed_data);
    PointCloudEncoder->encodePointCloud(output_cloud,compressed_data);

    std::cout << "Compressao" << std::endl;

    ROS_INFO("Tree depth: %d\n",PointCloudEncoder->getTreeDepth());

    compressed_data.seekg(0,ios::end);
    size = size + compressed_data.tellg();

    compressed_pointcloud_transport::CompressedPointCloud output_compressed;
    output_compressed.header = header->header;
    output_compressed.data = compressed_data.str();
    std::cout << "Headerrrrr" << std::endl;


    publish_pointcloud(output_compressed);
}

alfa_msg::AlfaConfigure::Response Alfa_Pc_Compress::process_config(alfa_msg::AlfaConfigure::Request &req)
{

}



void Alfa_Pc_Compress::do_Compression(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
{

    using namespace std::chrono;

    std::stringstream compressed_data_;

    ROS_INFO("Compressing cloud with frame [%s]", in_cloud->header.frame_id.c_str());
    this->in_cloud = in_cloud;

    auto start = high_resolution_clock::now();

    //compress point cloud
    //point_cloud_encoder->setInputCloud(in_cloud);
    point_cloud_encoder->encodePointCloud(in_cloud,compressed_data_);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    ROS_INFO("Compressing in %ld ms",duration.count());
    ROS_INFO("Tree depth: %d\n",point_cloud_encoder->getTreeDepth());

    // testes tempos
    tempos = tempos + duration.count();
    compressed_data_.seekg(0,ios::end);
    size = size + compressed_data_.tellg();
    size2 = size2 + static_cast<float> (in_cloud->size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f;
    x++;
    //test
    if(x==100){
      exe_time();
    }
}




void Alfa_Pc_Compress::exe_time()
{
    tempos = tempos/100 ;
    size = size/100;
    size2 = (size2*1000)/100;
    std::ofstream myFile("./output/exe_time");
    myFile<< "Exe. Time: "<< tempos << std::endl << "Point Cloud Size: "<< size2 << std::endl << "Compressed Size: "<<size<< std::endl << "Ratio: " << size2/size << std::endl ;
    myFile.close();
    std::cout << "-----------Acabei------------------------------------------------------------- " ;
    x=0;
    size = 0;
    size2 = 0;
}

//void Alfa_Pc_Compress::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
//{

//    if((input_cloud->width * input_cloud->height) == 0)
//        return; //return if the cloud is not dense

//    // convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointXYZI> for the encoder
//    try
//    {
//        pcl::fromROSMsg(*input_cloud, *in_cloud);
//    }
//    catch (std::runtime_error e)
//    {
//        ROS_ERROR_STREAM("Error in converting ros cloud to pcl cloud: " << e.what());
//    }

//    //do_Compression(in_cloud);
//    point_cloud_encoder->encodePointCloud(in_cloud,compressed_data);

//    compressed_pointcloud_transport::CompressedPointCloud output_compressed;
//    output_compressed.header= input_cloud->header;
//    output_compressed.data = compressed_data.str();

//    // Publish the data.
//    std::cout << "Publishing data" << std::endl;
//    pub.publish(output_compressed); // --------- > Nao publica direito Pq ?????
//    std::cout << "Data published" << std::endl;

//}
