#include "alfa_node.h"
#include <thread>
#include <unistd.h>
#include <chrono>


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

    if((cloud->width * cloud->height) == 0)
        return; //return if the cloud is not dense

    // convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointXYZI> for the encoder
    try
    {
        pcl::fromROSMsg(*cloud, *pcloud);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR_STREAM("Error in converting ros cloud to pcl cloud: " << e.what());
    }

    std::cout << "Recebi cloud" << std::endl;

    std::cout << (static_cast<float> (pcloud->size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f) << std::endl;

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
        newPing.config_tag = "DIOR software example";
        alfa_msg::ConfigMessage parameter1,parameter2,parameter3,parameter4,parameter5,parameter6;

        parameter1.config = 7;
        parameter1.config_name = "Filter Selector";

        parameter2.config = 0.1;
        parameter2.config_name = "Minimal Search Radius:";

        parameter3.config = 0.2;
        parameter3.config_name = "Multiplication Parameter:";

        parameter4.config = 5;
        parameter4.config_name = "Neighbor Threshold";

        parameter5.config = 0.005;
        parameter5.config_name = "Intensity Treshold Parameter:";

        parameter6.config = 4;
        parameter6.config_name = "Multithreading: Number of threads";

        newPing.default_configurations.push_back(parameter1);
        newPing.default_configurations.push_back(parameter2);
        newPing.default_configurations.push_back(parameter3);
        newPing.default_configurations.push_back(parameter4);
        newPing.default_configurations.push_back(parameter5);
        newPing.default_configurations.push_back(parameter6);

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







