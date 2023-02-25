#include "alfa_node.h"
#include <thread>
#include <unistd.h>
#include <chrono>
#include <cmath>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZI>);
#define RES_MULTIPLIER 100
#define INTENSITY_MULTIPLIER 1000


AlfaNode::AlfaNode()
{

    subscribe_topics();
    alive_ticker = new boost::thread(&AlfaNode::ticker_thread,this);
    pcloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

}

void AlfaNode::publish_pointcloud(compressed_pointcloud_transport::CompressedPointCloud pcl2_frame)
{

//    cout << pcl2_frame.header << endl;
    cloud_publisher.publish(pcl2_frame);
    cout << "Published compressed point cloud"<<endl;

}

void AlfaNode::publish_metrics(alfa_msg::AlfaMetrics &metrics)
{
    filter_metrics.publish(metrics);
}

void AlfaNode::process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud,const sensor_msgs::PointCloud2ConstPtr& header)
{
    cout << "Please implement the process_pointcloud function"<<endl;
}

alfa_msg::AlfaConfigure::Response AlfaNode::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    cout << "Please implement the process_config function"<<endl;

}

AlfaNode::~AlfaNode()
{

}

void AlfaNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{

    std::vector<char> vec;
    std::vector<float> vecx;
    std::vector<float> vecy;
    std::vector<float> vecz;
    int i=0;
    int j=0;

    if((cloud->width * cloud->height) == 0)
        return; //return if the cloud is not dense

    // convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointXYZI> for the encoder
    try
    {
        pcl::fromROSMsg(*cloud, *pcloud);
        //pcl::fromROSMsg(*cloud, *cloud_test);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR_STREAM("Error in converting ros cloud to pcl cloud: " << e.what());
    }

    std::cout << "Recebi cloud\n" << std::endl;

    std::cout << (static_cast<float> (pcloud->size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f) << std::endl;

    printf("Point Cloud Size: %d\n",pcloud->size());

   process_pointcloud(pcloud,cloud);


}

bool AlfaNode::parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res)
{
    cout<<"Recieved FilterSettings with size" <<req.configurations.size()<<"... Updating"<<endl;
    for (int i=0; i< req.configurations.size();i++) {
        cout <<"Configuration: "<<i<< " With name: "<< req.configurations[i].config_name<< " with value: "<< req.configurations[i].config<<endl;
    }

    res = process_config(req);
    return true;
}

void AlfaNode::init()
{
        char arg0[]= "filter_node";
        char *argv[]={arg0,NULL};
        int argc=(int)(sizeof(argv) / sizeof(char*)) - 1;;
        ros::init (argc, argv, NODE_NAME);
          if (!ros::master::check()) {
              cout <<"Failed to inicialize ros"<<endl;
            return;
          }

}

void AlfaNode::subscribe_topics()
{
    sub_cloud = nh.subscribe("alfa_pointcloud",1,&AlfaNode::cloud_cb,this);
    sub_parameters = nh.advertiseService(string(NODE_NAME).append("_settings"),&AlfaNode::parameters_cb,this);
    ros::NodeHandle n;
    filter_metrics = n.advertise<alfa_msg::AlfaMetrics>(string(NODE_NAME).append("_metrics"), 1);
    alive_publisher = n.advertise<alfa_msg::AlfaAlivePing>(string(NODE_NAME).append("_alive"),1);
    cloud_publisher = n.advertise<compressed_pointcloud_transport::CompressedPointCloud>("point_cloud/compressed",1);
    //m_spin_thread = new boost::thread(&AlfaNode::spin, this);


}

void AlfaNode::ticker_thread()
{
    while(ros::ok())
    {
        alfa_msg::AlfaAlivePing newPing;
        newPing.node_name= NODE_NAME;
        newPing.node_type = NODE_TYPE;
        newPing.config_service_name = string(NODE_NAME)+"_settings";
        newPing.config_tag = "ALFA-Pc example";
        alfa_msg::ConfigMessage parameter1,parameter2,parameter3,parameter4,parameter5,parameter6;

        parameter1.config = 0.03;
        parameter1.config_name = "Resolution";

        parameter2.config = 0;
        parameter2.config_name = "Multithreading";

        parameter3.config = 32;
        parameter3.config_name = "Hw test - Number of points";

        parameter4.config = 10;
        parameter4.config_name = "Octreee Depth Hw";

        newPing.default_configurations.push_back(parameter1);
        newPing.default_configurations.push_back(parameter2);
        newPing.default_configurations.push_back(parameter3);
        newPing.default_configurations.push_back(parameter4);


        newPing.current_status = node_status;
        alive_publisher.publish(newPing);
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMER_SLEEP));
    }
}

void AlfaNode::spin()
{
    cout<<"started spinning with success"<<endl;
    ros::spin();
}



//    for (auto point : cloud_test->points) {
//        std::cout << point._PointXYZI::intensity << std::endl;
//        vec.push_back(point._PointXYZI::intensity);
//        vecx.push_back(point._PointXYZI::x);
//        vecy.push_back(point._PointXYZI::y);
//        vecz.push_back(point._PointXYZI::z);
//    }



//    for(auto point: pcloud->points){
//        std::cout << "----------------------------------" <<vec[j] <<  std::endl;
//        point._PointXYZRGB::r = vec[j];
//        point._PointXYZRGB::x = vecx[j];
//        point._PointXYZRGB::y = vecy[j];
//        point._PointXYZRGB::z = vecz[j];
//        j++;
//    }

//   pcloud->header = cloud_test->header;

void AlfaNode::store_pointcloud_hardware(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, u64 *pointer)
{
    int pointcloud_index = 0;
    int16_t a16_points[4];
    for (auto point :*input_cloud) {
        a16_points[0] = point.x*RES_MULTIPLIER;
        a16_points[1] = point.y*RES_MULTIPLIER;
        a16_points[2] = point.z*RES_MULTIPLIER;
        a16_points[3] =0;//point.intensity*INTENSITY_MULTIPLIER;
        memcpy((void*)(pointer+pointcloud_index),a16_points,sizeof(int16_t)*4);
        //printf("Point %d | (%d , %d , %d )\n",pointcloud_index,a16_points[0],a16_points[1],a16_points[2]);
        pointcloud_index++;
    }
}


std::vector<char> AlfaNode::read_hardware_pointcloud(u64 *pointer, uint size)
{
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr return_cloud;
    //return_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    

    std::vector<char> hw_occupancy_code;
    uint32_t counter = 0;
    float number_bits = 0;
    int ddr_blocks = 0;
    float blocks = 0;

    number_bits = size*8;
    blocks = number_bits/64.0;
    ddr_blocks = ceil(blocks);

    printf("DDR Bits to Read: %f\n",number_bits);
    printf("DDR Blocks: %f\n",blocks);
    printf("DDR Blocks to Read: %d\n",ddr_blocks);

    for(int i=0;i<ddr_blocks;i++){
        uint8_t a8_branchs[8];
        memcpy((void*)(a8_branchs), pointer+i,sizeof(uint8_t)*8);
        hw_occupancy_code.push_back(a8_branchs[7]);
        hw_occupancy_code.push_back(a8_branchs[6]);
        hw_occupancy_code.push_back(a8_branchs[5]);
        hw_occupancy_code.push_back(a8_branchs[4]);
        hw_occupancy_code.push_back(a8_branchs[3]);
        hw_occupancy_code.push_back(a8_branchs[2]);
        hw_occupancy_code.push_back(a8_branchs[1]);
        hw_occupancy_code.push_back(a8_branchs[0]);
          /* for(int j=7;j>=0;j--){
            if(counter < size){
                hw_occupancy_code.push_back(a8_branchs[j]);
                counter++; 
            }else{ 
                break;
            }
        } */
    }

//    63                             0
//    [4d][80][80][88][80][80][80][20]

//    for(int i = 0;i<hw_occupancy_code.size();i++){
//        printf("%x\n",hw_occupancy_code[i]);
//    }

    return hw_occupancy_code;

    /* for (uint i=0; i<size;i++) {
        pcl::PointXYZRGB p;
        int16_t a16_points[4];
        memcpy((void*)(a16_points), pointer+i,sizeof(int16_t)*4);
        p.x = (a16_points[0])/float(RES_MULTIPLIER);
        p.y = (a16_points[1])/float(RES_MULTIPLIER);
        p.z = (a16_points[2])/float(RES_MULTIPLIER);
        //p.intensity = (a16_points[3])/float(INTENSITY_MULTIPLIER);
        return_cloud->push_back(p);
        #ifdef DEBUG

        cout<< "First bits: "<< hex<< a16_points[0]<< " Secound bits: "<< hex<< a16_points[1]<<endl;
        cout << "Obtained coordinate: X:"<< hex<< p.x<< "; Y: "<<hex <<p.y<< "; Z: "<<hex<<p.z<< "; Intensity: "<<p.intensity<<endl;
        #endif

    }
    return return_cloud; */
}

vector<uint32_t> AlfaNode::read_hardware_registers(uint32_t *pointer, uint size)
{
    vector<uint32_t> return_vector;
    for (uint var = 0; var < size; ++var) {
        return_vector.push_back(pointer[var]);
    }
    return return_vector;
}

void AlfaNode::write_hardware_registers(vector<uint32_t> data, uint32_t *pointer, uint offset)
{
    for(uint i = offset; i <data.size(); i++)
    {
        pointer[i] = data[i];
    }
}
