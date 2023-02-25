#include "ros/ros.h"
#include "compressor.h"

int main(int argc, char **argv)
{

     ros::init (argc, argv, "compression_node_mt");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }

    Alfa_Pc_Compress new_node;
    while(ros::ok())
    {

    }
}
