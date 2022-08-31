#include <ros/ros.h>


// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include "alfa_msg/AlfaConfigure.h"
#include "alfa_msg/AlfaMetrics.h"
#include "alfa_msg/AlfaAlivePing.h"

#include "CompressedPointCloud.h"

#define NODE_NAME "alfa_pc_compression_node"

#define NODE_TYPE "Compression"


#define TIMER_SLEEP 50000

typedef long long int u64;

using namespace std;
class AlfaNode
{
public:
    AlfaNode();

    void publish_pointcloud(compressed_pointcloud_transport::CompressedPointCloud output_cloud);
    void publish_metrics(alfa_msg::AlfaMetrics &metrics);

    virtual void process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, const sensor_msgs::PointCloud2ConstPtr& header);
    virtual alfa_msg::AlfaConfigure::Response   process_config(alfa_msg::AlfaConfigure::Request &req);

    int node_status;
    virtual ~AlfaNode();
    uint pcl2_header_seq;
    void spin();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud;

    virtual void store_pointcloud_hardware(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, u64 *pointer);
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr  read_hardware_pointcloud(u64 *pointer, uint size);
    virtual vector<uint32_t> read_hardware_registers(uint32_t* pointer, uint size);
    virtual void  write_hardware_registers(vector<uint32_t>  data, uint32_t* pointer, uint offset = 0);

private:

    void cloud_cb (const  sensor_msgs::PointCloud2ConstPtr& cloud);
    bool parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res);
    ros::Subscriber sub_cloud;
    ros::ServiceServer sub_parameters;
    ros::NodeHandle nh;

    void init();
    void subscribe_topics();
    void ticker_thread();
    boost::thread *m_spin_thread;
    ros::Publisher filter_metrics;
    ros::Publisher alive_publisher;
    ros::Publisher cloud_publisher;

    boost::thread* alive_ticker;



};





