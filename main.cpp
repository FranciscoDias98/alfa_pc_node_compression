#include "ros/ros.h"
#include "compressor.h"

int main(int argc, char **argv)
{

     ros::init (argc, argv, "alfa_pc_compression_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }

    Alfa_Pc_Compress new_node;
    while(ros::ok())
    {

    }
}
