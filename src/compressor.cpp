#include "compressor.h"
#include "CompressedPointCloud.h"
#include "octree_pointcloud_compression_2.h"
//#include "my_octree.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#include <pcl/octree/octree_base.h>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

#include <chrono>
#include <time.h>
#include <string>
#include<unistd.h>
#include <fstream>
// testes tempos
unsigned int x = 0;
unsigned long tempos = 0;
float size_compressed =0;
float size_original =0;

unsigned long tempos_test = 0;
float size_compressed_test =0;
float size_original_test =0;
float points_second = 0;

bool oct_0,oct_1,oct_2,oct_3,oct_4,oct_5,oct_6,oct_7;

pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder1;



pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_0;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_1;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_2;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_3;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_4;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_5;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_6;
pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder_7;


vector<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>> octants_compressor_vec;
vector<double>x_;
vector<double>y_;
vector<double>z_;
vector<pcl::PointXYZ> points;





Alfa_Pc_Compress::Alfa_Pc_Compress()
{
    std::cout << " ---------- ALFA-Pc Compressor Constructor -----------" << std::endl;

    //-------------- SW-HW Memory Init ---------------------

    unsigned int region_size = 0x10000;
    off_t axi_pbase = 0xA0000000;
    //u_int32_t *hw32_vptr;
    //u64 *ddr_pointer;
    int fd;
    unsigned int ddr_size = 0x200000;
    off_t ddr_ptr_base = 0x0F000000; // physical base address
    //Map the physical address into user space getting a virtual address for it
    hw = 0;

    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
        ddr_pointer = (u64 *)mmap(NULL, ddr_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_ptr_base);
        hw32_vptr = (u_int32_t *)mmap(NULL, region_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, axi_pbase);
        hw = 1;
    }
    else
        ROS_INFO("NAO ENTROU NO NMAP :(");

    //--------------------------------------------------------//



    vector<char>vector;



    in_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    out_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    cluster0.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cluster1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);



    octant_0.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_3.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_4.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_5.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_6.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_7.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    octree_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);




    compressed_data_vector.clear();



    /////////////////// added for multihreading
    multi_thread = false;
    test = false;
    number_threads = 8;

    frame_header_identifier= "<PCL-OCT-COMPRESSED>";






    set_compression_profile(); // define compression profile
    spin();






}

void Alfa_Pc_Compress::set_compression_profiles_octants()
{
    compression_profile = pcl::io::MANUAL_CONFIGURATION;

    show_statistics = true;
    compression_profile_.pointResolution = 0.01; //
    compression_profile_.octreeResolution = 0.5; //-----> ALTERAR NESTE!!!!!!!!! voxel size in cubic meters (1m is 0.01 cm)
    compression_profile_.doVoxelGridDownSampling = true;
    compression_profile_.iFrameRate = 10; // number of prediction frames
    compression_profile_.colorBitResolution = 0;
    compression_profile_.doColorEncoding = false;

    PointCloudEncoder_0 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
    PointCloudEncoder_1 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
    PointCloudEncoder_2 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
    PointCloudEncoder_3 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
    PointCloudEncoder_4 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
    PointCloudEncoder_5 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
    PointCloudEncoder_6 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);
    PointCloudEncoder_7 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);



}



void Alfa_Pc_Compress::set_compression_profile()
{
    std::cout << "Setting Compression Profile" << std::endl;

    if(multi_thread)
        set_compression_profiles_octants();
    else{

    compression_profile = pcl::io::MANUAL_CONFIGURATION;

    show_statistics = true;
    compression_profile_.pointResolution = 0.01; //
    compression_profile_.octreeResolution = 0.03; //-----> ALTERAR NESTE!!!!!!!!! voxel size in cubic meters (1m is 0.01 cm)
    compression_profile_.doVoxelGridDownSampling = true;
    compression_profile_.iFrameRate = 0; // number of prediction frames
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


    PointCloudEncoder1 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,
                        show_statistics,
                        compression_profile_.pointResolution,
                        compression_profile_.octreeResolution,
                        compression_profile_.doVoxelGridDownSampling,
                        compression_profile_.iFrameRate,
                        compression_profile_.doColorEncoding,
                        compression_profile_.colorBitResolution);


    }



    //octants_compressor_vec[0] = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>;





    output_metrics.message_tag = "Compression performance";

    std::cout << "End Setting Compression Profile" << std::endl;
}




void Alfa_Pc_Compress::process_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, const sensor_msgs::PointCloud2ConstPtr& header)
{
    std::cout << "Entrei Process Cloud" << std::endl;

    //////// octree test viewer //////////////////////////////////////
//    octree_cloud->width = input_cloud->size();
//    octree_cloud->height=1;
//    octree_cloud->points.resize (octree_cloud->width * octree_cloud->height);
//    for(int i=0;i<input_cloud->size();i++){
//        octree_cloud->points[i].x = input_cloud->points[i].x;
//        octree_cloud->points[i].y = input_cloud->points[i].y;
//        octree_cloud->points[i].z = input_cloud->points[i].z;
//    }

//    pcl::io::savePCDFileASCII ("frame1_hdl32.pcd", *octree_cloud);
    /////////////////////////////////////////////////////////////////

    this->in_cloud = input_cloud;
    octant_0->clear();
    octant_1->clear();
    octant_2->clear();
    octant_3->clear();
    octant_4->clear();
    octant_5->clear();
    octant_6->clear();
    octant_7->clear();

    std::cout << "PAssei Clears" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud;
    test_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::stringstream compressed_data;
    //std::stringstream compressed_data_2;
    output_compressed.header = header->header;
    output_compressed1.header = header->header;
    //this->header = header;
    output_metrics.metrics.clear();

    std::cout << "PAssei stringstream" << std::endl;

    printf("Point Cloud Size: %d\n",input_cloud->size());


    ROS_INFO("Compressing cloud with frame [%s]\n", input_cloud->header.frame_id.c_str());
    ROS_INFO("Compressing cloud with frame [%d]\n", input_cloud->header.seq);

    std::cout << "PAssei ROS Info" << std::endl;

    auto start_exe_time = high_resolution_clock::now();


    if(multi_thread)
    {
        auto start_multi = high_resolution_clock::now();


        auto start_divide = high_resolution_clock::now();
        //divide_in_octants_test(input_cloud); // <---------------------- Check Speed ????? divide_in_octants
        divide_in_octants_test(input_cloud);
        auto stop_divide = high_resolution_clock::now();
        auto duration_divide = duration_cast<milliseconds>(stop_divide - start_divide);
        ROS_INFO("------ Octant Division in %ld ms ----- ",duration_divide.count());

        if(thread_list.size()>1)
        {
            for (int i =0;i < thread_list.size();i++)
            {
                thread_list[i]->join();
            }

                thread_list.clear();
        }

        if (number_threads >1)
        {
            thread_list.clear();
            for (int i =0;i < number_threads;i++)
            {
                //thread_list.push_back(new boost::thread(&Alfa_Pc_Compress::run_worker_thread, this,i,input_cloud));

                // multithread odd/even frames
                thread_list.push_back(new boost::thread(&Alfa_Pc_Compress::run_worker_thread_octants, this,i,input_cloud));
                //thread_list.push_back(new boost::thread(&Alfa_Pc_Compress::divide_in_octants_multi, this,i,input_cloud));
            }
            for (int i =0;i < number_threads;i++)
            {
                thread_list[i]->join();
            }
        }

        printf("------ Acabei Threads ------ \n");

        while(!(oct_0 && oct_1 && oct_2 && oct_3 && oct_4 && oct_5 && oct_6 && oct_7))
            printf("------ Waiting ------ \n");

        oct_0 = false;
        oct_1 = false;
        oct_2 = false;
        oct_3 = false;
        oct_4 = false;
        oct_5 = false;
        oct_6 = false;
        oct_7 = false;


        auto write_start = high_resolution_clock::now();
        //write octants information in header
        my_write_frame_header(compressed_data);
        auto write_stop = high_resolution_clock::now();
        auto write_duration = duration_cast<milliseconds>(write_stop - write_start);
        ROS_INFO("------ Write Duration in %ld ms ----- ",write_duration.count());

        //merge occupancy_codes off octants
        auto merge_start = high_resolution_clock::now();

        //std::vector<char> occupancy_codes;
        //occupancy_codes.insert(occupancy_codes.end(),PointCloudEncoder_0->binary_tree_data_vector_.begin(),PointCloudEncoder_0->binary_tree_data_vector_.end());
//        for(int i=0; i<PointCloudEncoder_0->binary_tree_data_vector_.size();i++)
//            printf("%i --> %x \n",i,PointCloudEncoder_0->binary_tree_data_vector_[i]);

        printf("\n------------------------------------------------------------\n");
        PointCloudEncoder_0->binary_tree_data_vector_.insert(PointCloudEncoder_0->binary_tree_data_vector_.end(),PointCloudEncoder_1->binary_tree_data_vector_.begin(),PointCloudEncoder_1->binary_tree_data_vector_.end());
        PointCloudEncoder_0->binary_tree_data_vector_.insert(PointCloudEncoder_0->binary_tree_data_vector_.end(),PointCloudEncoder_2->binary_tree_data_vector_.begin(),PointCloudEncoder_2->binary_tree_data_vector_.end());
        PointCloudEncoder_0->binary_tree_data_vector_.insert(PointCloudEncoder_0->binary_tree_data_vector_.end(),PointCloudEncoder_3->binary_tree_data_vector_.begin(),PointCloudEncoder_3->binary_tree_data_vector_.end());
        PointCloudEncoder_0->binary_tree_data_vector_.insert(PointCloudEncoder_0->binary_tree_data_vector_.end(),PointCloudEncoder_4->binary_tree_data_vector_.begin(),PointCloudEncoder_4->binary_tree_data_vector_.end());
        PointCloudEncoder_0->binary_tree_data_vector_.insert(PointCloudEncoder_0->binary_tree_data_vector_.end(),PointCloudEncoder_5->binary_tree_data_vector_.begin(),PointCloudEncoder_5->binary_tree_data_vector_.end());
        PointCloudEncoder_0->binary_tree_data_vector_.insert(PointCloudEncoder_0->binary_tree_data_vector_.end(),PointCloudEncoder_6->binary_tree_data_vector_.begin(),PointCloudEncoder_6->binary_tree_data_vector_.end());
        PointCloudEncoder_0->binary_tree_data_vector_.insert(PointCloudEncoder_0->binary_tree_data_vector_.end(),PointCloudEncoder_7->binary_tree_data_vector_.begin(),PointCloudEncoder_7->binary_tree_data_vector_.end());






        auto merge_stop = high_resolution_clock::now();
        auto merge_duration = duration_cast<milliseconds>(merge_stop - merge_start);
        ROS_INFO("------ Merge Duration in %ld ms ----- ",merge_duration.count());

        printf("Binary Tree Data Vector Size: %d bytes\n",PointCloudEncoder_0->binary_tree_data_vector_.size());

        auto compress_merged_start = high_resolution_clock::now();
        PointCloudEncoder_0->my_entropyEncoding(compressed_data);   // Vai ser aqui para mandar para o HW <-----------
        auto compress_merged_stop = high_resolution_clock::now();
        auto compress_merged_duration = duration_cast<milliseconds>(compress_merged_stop - compress_merged_start);
        ROS_INFO("------ Compress Merged Duration in %ld ms ----- ",compress_merged_duration.count());

        PointCloudEncoder_0->switchBuffers();
        PointCloudEncoder_0->object_count_ = 0;
        PointCloudEncoder_1->switchBuffers();
        PointCloudEncoder_1->object_count_ = 0;
        PointCloudEncoder_2->switchBuffers();
        PointCloudEncoder_2->object_count_ = 0;
        PointCloudEncoder_3->switchBuffers();
        PointCloudEncoder_3->object_count_ = 0;
        PointCloudEncoder_4->switchBuffers();
        PointCloudEncoder_4->object_count_ = 0;
        PointCloudEncoder_5->switchBuffers();
        PointCloudEncoder_5->object_count_ = 0;
        PointCloudEncoder_6->switchBuffers();
        PointCloudEncoder_6->object_count_ = 0;
        PointCloudEncoder_7->switchBuffers();
        PointCloudEncoder_7->object_count_ = 0;



        //********************************************************************//
        auto stop_multi = high_resolution_clock::now();
        auto duration_multi = duration_cast<milliseconds>(stop_multi - start_multi);
        ROS_INFO("------ Multi Thread Compression in %ld ms ----- ",duration_multi.count());
        tempos_test = tempos_test + duration_multi.count();
        x++;
        if(x==100){
            x=0;
            tempos_test = tempos_test/100 ;
            ROS_INFO("************************** Multi Thread Compression Average in %ld ms ********************************************************************** ",tempos_test);
            tempos_test = 0;
            //exe_time();
        }

        output_compressed.header = header->header;
        output_compressed.data = compressed_data.str();
        publish_pointcloud(output_compressed);





    }else if(test){
        printf("Test\n");


        // ************ Encode Point Cloud - Sw *********************

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud;
        test_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_output_cloud;
        test_output_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        for(int i=0;i<16000;i++){
             pcl::PointXYZRGB point = (*input_cloud)[i];
             test_cloud->push_back(point);
        }
        //pcl::io::savePCDFile("test_pcd.pcd", *test_cloud);


        //pcl::io::savePCDFile("test_pcd.pcd", *test_cloud);

        //PointCloudEncoder1->setResolution(0.125);
        //PointCloudEncoder->setTreeDepth(13);
        //PointCloudEncoder1->defineBoundingBox(-128,-128,-128,128,128,128);
        //PointCloudEncoder1->encodePointCloud_2(test_cloud,compressed_data);

        // %%%%%%%%%%%%%%%%%%%%% PRINT OCTREE OCC. CODE %%%%%%%%%%%%%%%%%%%%%%%%%%

//        FILE *fp_binary_tree;
//        fp_binary_tree = fopen("binary_tree_bfs_sw_.txt","wb");
//        for(int i=0;i<PointCloudEncoder1->binary_tree_data_vector_.size();i++){
//            fprintf(fp_binary_tree,"%x   | ",(unsigned char)(PointCloudEncoder1->binary_tree_data_vector_[i]));
//            for (int j = sizeof(char) * 7; j >= 0; j--)
//                    fprintf(fp_binary_tree,"%d", ((PointCloudEncoder1->binary_tree_data_vector_[i]) & (1 << j)) >> j );
//            fprintf(fp_binary_tree,"\n");

//        }
//        fclose(fp_binary_tree);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        // %%%%%%%%%%%%%%%%%%%%% PRINT OCTREE STRUCTURE %%%%%%%%%%%%%%%%%%%%%%%%%%

        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        //printf("Leaf Count: %d\n",PointCloudEncoder1->getLeafCount());
        //PointCloudEncoder1->decodePointCloud(compressed_data,test_output_cloud);
        //pcl::io::savePCDFile("output_cloud_test_normal_sw.pcd", *test_output_cloud);





        // **********************************************************


        //PointCloudEncoder->getInputCloud();
        compressed_data.clear();
        std::cout << "Entrei Test" << std::endl;

        // decompress test
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud;
        output_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        //********** START DECODING OCCUPANCY CODE ***************

        FILE *fp_occ_test;
        fp_occ_test = fopen("occ_code_test.txt","rb");
        std::string line;

        std::ifstream file("occ_code_test.txt");
        vector<char> occupancy_code_test;
        while (getline(file, line)) {
              char value = (char) std::stoi(line, nullptr, 16);
              occupancy_code_test.push_back(value);
            }
        file.close();


        PointCloudEncoder->setInputCloud(test_cloud);
        printf("Input Cloud Test Size: %d\n",test_cloud->size());

        // increase frameID
        PointCloudEncoder->frame_ID_++;
        if (!PointCloudEncoder->do_voxel_grid_enDecoding_)
        {
         PointCloudEncoder->point_count_data_vector_.clear ();
         PointCloudEncoder->point_count_data_vector_.reserve (test_cloud->points.size ());
        }
        // initialize point encoding
        PointCloudEncoder->point_coder_.initializeEncoding ();
        PointCloudEncoder->point_coder_.setPointCount (static_cast<unsigned int> (test_cloud->points.size ()));
        PointCloudEncoder->binary_tree_data_vector_ = occupancy_code_test;
        printf("Binary Tree Size: %d\n",PointCloudEncoder->binary_tree_data_vector_.size());
        printf("Point Count %d \n",PointCloudEncoder->point_count_);
        printf("Leaf Count %d \n",PointCloudEncoder->getLeafCount());
        printf("Object Count %d \n",PointCloudEncoder->object_count_);
        PointCloudEncoder->setResolution(0.125);
        //my_write_frame_header_hw(compressed_data);

        //PointCloudEncoder->entropyEncoding(compressed_data);
        PointCloudEncoder->switchBuffers ();

        //output_compressed.header = header->header;
        //output_compressed.data = compressed_data.str();
        //publish_pointcloud(output_compressed);


        // %%%%%%%%%%%%%%%%%% DECODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        PointCloudEncoder->switchBuffers();
        PointCloudEncoder->setOutputCloud(output_cloud);
        PointCloudEncoder->cloud_with_color_ = false;
        PointCloudEncoder->deleteTree();
        PointCloudEncoder->defineBoundingBox(-256,-256,-256,256,256,256);
        printf("Leaf Count: %d\n",PointCloudEncoder->getLeafCount());
        PointCloudEncoder->point_coder_.setPrecision (static_cast<float> (0.01));
        PointCloudEncoder->point_coder_.initializeDecoding ();
        PointCloudEncoder->output_->points.clear ();
        PointCloudEncoder->output_->points.reserve (static_cast<std::size_t> (PointCloudEncoder->point_count_));
        PointCloudEncoder->setResolution(0.06);
        PointCloudEncoder->setTreeDepth(13);
        PointCloudEncoder->deserializeTree(PointCloudEncoder->binary_tree_data_vector_,false);
        // assign point cloud properties
        PointCloudEncoder->output_->height = 1;
        PointCloudEncoder->output_->width = static_cast<uint32_t>(output_cloud->points.size ());
        PointCloudEncoder->output_->is_dense = false;



        printf("%%%%%%%%%%%%%%%%%% RESULTS %%%%%%%%%%%%%%%%%%%%%%%\n");
        printf("Input Cloud Size Points: %d\n",test_cloud->size());

        printf(" --> SW:\n");
        printf("     Output Cloud Size: %d Points\n",test_output_cloud->size());

        printf(" --> HW:\n");
        printf("     Output Cloud Test Size: %d Points\n",output_cloud->size());





        pcl::PointCloud<pcl::PointXYZRGB>::Ptr marosca;
        marosca.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i=0;i<output_cloud->size();i++) {
            pcl::PointXYZRGB point = (*output_cloud)[i];
            point.x = (point.x)*0.5;
            point.y = (point.y)*0.5;
            point.z = (point.z)*0.5;
            marosca->push_back(point);
        }

        pcl::io::savePCDFile("output_cloud_hw.pcd", *output_cloud);
        pcl::io::savePCDFile("marosca_hw.pcd", *marosca);




        //output_compressed.header = header->header;
        //output_compressed.data = compressed_data.str();
        //publish_pointcloud(output_compressed);


    } else if (hw) {

        //test ( remove this, monitor depth/resolution)
        // test with 7 points, the same as testbench

        uint32_t depth = 5;
        uint32_t hw_leaf_count = 0;
        uint32_t hw_branch_count = 0;



        for(int i=0;i<size_test;i++){
             pcl::PointXYZRGB point = (*input_cloud)[i];
             test_cloud->push_back(point);
        } //  40532


        // ************ Encode Point Cloud - Hw *********************
        //store point cloud in Hw
        printf("************ Storing Point Cloud ****************\n");
        auto start_store_hw = std::chrono::high_resolution_clock::now();
        store_pointcloud_hardware(input_cloud,ddr_pointer);
        auto stop_store_hw = std::chrono::high_resolution_clock::now();
        usleep(10);
        printf("*************************************************\n");

        //octree_core
        vector<uint32_t> configs;
        configs.clear();
        configs.push_back(1);
        configs.push_back(input_cloud->size());
        configs.push_back(0);
        configs.push_back(depth_test);
        write_hardware_registers(configs, hw32_vptr);

         int hardware_finish = 0;
         int value = 0;
         while(!hardware_finish){
            vector<uint32_t> hardware_result = read_hardware_registers(hw32_vptr, 6);
            value = hardware_result[2];
            if(value==1){
                hardware_finish = 1;
                hw_leaf_count = hardware_result[4];
                hw_branch_count = hardware_result[5];
            }else
                usleep(1);
        }
        printf("************ Compressing Point Cloud ****************\n");
        printf("Number of Points: %d\n",input_cloud->size());
        printf("Octree Depth: %d\n",depth_test);

        vector<char> occupancy_code_hw;
        //read hw occ_code
        auto start_read_hw = std::chrono::high_resolution_clock::now();
        occupancy_code_hw = read_hardware_pointcloud(ddr_pointer,hw_branch_count);
        auto stop_read_hw = std::chrono::high_resolution_clock::now();
        //


        //FILE *fp;
        //fp = fopen("occ_code_hw.txt","wb");

        //PointCloudEncoder->decodePointCloud(occupancy_code_hw,output_cloud);

        auto duration_store_hw = std::chrono::duration_cast<std::chrono::milliseconds>(stop_store_hw - start_store_hw);
        auto duration_read_hw = std::chrono::duration_cast<std::chrono::microseconds>(stop_read_hw - start_read_hw);
        cout << "STORE TIME: " << duration_store_hw.count() << "ms" << endl;
        cout << "READ TIME: " << duration_read_hw.count() << "us" << endl;
        printf("Hw Leaf Count: %d\n",hw_leaf_count);
        printf("Hw Branch Count: %d\n",hw_branch_count);
//        printf("************ Occ. Code ****************\n");
//        for(int i = 0; i<occupancy_code_hw.size();i++){
//            //printf("Id: %d | Occ. Code: %x \n",i,occupancy_code_hw[i]);
//            fprintf(fp,"Id: %d | Occ. Code: %x \n",i,occupancy_code_hw[i]);
//        }
//        fclose(fp);
        printf("*************************************************\n");


        // decompress test
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud;

        //********** START DECODING OCCUPANCY CODE ***************

        PointCloudEncoder->setInputCloud(input_cloud);
        printf("Input Cloud Test Size: %d\n",input_cloud->size());

        // increase frameID
        PointCloudEncoder->frame_ID_++;
        if (!PointCloudEncoder->do_voxel_grid_enDecoding_)
        {
         PointCloudEncoder->point_count_data_vector_.clear ();
         PointCloudEncoder->point_count_data_vector_.reserve (test_cloud->points.size ());
        }
        // initialize point encoding
        PointCloudEncoder->point_coder_.initializeEncoding ();
        PointCloudEncoder->point_coder_.setPointCount (static_cast<unsigned int> (input_cloud->points.size ()));
        PointCloudEncoder->binary_tree_data_vector_ = occupancy_code_hw;
        printf("Binary Tree Size: %d\n",PointCloudEncoder->binary_tree_data_vector_.size());
        printf("Point Count %d \n",PointCloudEncoder->point_count_);
        printf("Leaf Count %d \n",PointCloudEncoder->getLeafCount());
        printf("Object Count %d \n",PointCloudEncoder->object_count_);

        switch (depth_test) {
            case 15: PointCloudEncoder->setResolution(0.03);
                break;
            case 14: PointCloudEncoder->setResolution(0.06);
                break;
            case 13: PointCloudEncoder->setResolution(0.125);
                break;
            case 12: PointCloudEncoder->setResolution(0.25);
                break;
            case 11: PointCloudEncoder->setResolution(0.5);
                break;
            case 10: PointCloudEncoder->setResolution(1);

        }

        //PointCloudEncoder->setResolution(0.125);

        my_write_frame_header_hw(compressed_data);
        auto start_read_rangeEncoder = std::chrono::high_resolution_clock::now();
        PointCloudEncoder->entropyEncoding(compressed_data);
        auto stop_read_rangeEncoder = std::chrono::high_resolution_clock::now();
        PointCloudEncoder->switchBuffers ();


        auto duration_compress = std::chrono::duration_cast<std::chrono::milliseconds>(stop_read_rangeEncoder - start_read_rangeEncoder);
        cout << "Compress TIME: " << duration_compress.count() << "ms" << endl;






        output_compressed.header = header->header;
        output_compressed.data = compressed_data.str();
        publish_pointcloud(output_compressed);
        //**********************************************



        // **********************************************************

    }
    else{

        // compress
        auto start = high_resolution_clock::now();

        //do_Compression(output_cloud);





        //divide_in_octants(input_cloud);
        PointCloudEncoder->encodePointCloud_2(input_cloud,compressed_data);

        ////////////// octree tests ////////////////////
        char *ptr = 0;

        FILE *fp1;
        fp1 = fopen("Octree_memory.txt", "wb");

        for(int i=0;i<PointCloudEncoder->binary_tree_data_vector_.size();i++){
            ptr = &PointCloudEncoder->binary_tree_data_vector_[i];
            fprintf(fp1,"________________________\n");
            fprintf(fp1,"%i | %x | %x",i,ptr,PointCloudEncoder->binary_tree_data_vector_[i]);
        }
        fclose(fp1);

//        for(int i=0;i<PointCloudEncoder->binary_tree_data_vector_.size();i++){
//            ptr = PointCloudEncoder->binary_tree_data_vector_[i];
//        }



        //////////////////////////////////////////////////////
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        std::cout << "Fiz Compressao" << std::endl;
        size_original_test = size_original_test + (static_cast<float> (input_cloud->points.size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;

        compressed_data.seekg(0,ios::end);
        size_compressed_test = size_compressed_test+compressed_data.tellg();
        tempos_test = tempos_test + duration.count();
        points_second += 1000*input_cloud->points.size() / duration.count();
        x++;
        if(x==100){
            x=0;
            exe_time();
        }
        points_second += 1000*input_cloud->points.size() / duration.count();
        ROS_INFO("Compressing in %ld ms",duration.count());



        // push metrics to monitor
        metrics(compressed_data,input_cloud,duration.count());


        // pub compressed
        output_compressed.header = header->header;
        output_compressed.data = compressed_data.str();
        publish_pointcloud(output_compressed);

    }


    auto stop_exe_time = high_resolution_clock::now();
    auto duration_exe_time = duration_cast<milliseconds>(stop_exe_time - start_exe_time);
    ROS_INFO("------ Compressing in %ld ms ----- ",duration_exe_time.count());

    std::cout << "Passed publish_pointcloud " << std::endl;
    metrics(compressed_data,test_cloud,duration_exe_time.count());

    publish_metrics(output_metrics);
}

void Alfa_Pc_Compress::my_write_frame_header_hw(ostream &compressed_tree_data_out_arg)
{
    compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (frame_header_identifier), strlen (frame_header_identifier));
    // encode point cloud header id
    compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder->frame_ID_), sizeof (PointCloudEncoder->frame_ID_));
    // encode frame type (I/P-frame)
    compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder->i_frame_), sizeof (PointCloudEncoder->i_frame_));


    if (PointCloudEncoder->i_frame_)
    {
      printf("i_frame\n");
      double min_x, min_y, min_z, max_x, max_y, max_z;
      double octree_resolution;
      unsigned char color_bit_depth;
      double point_resolution;

      // get current configuration
      octree_resolution = PointCloudEncoder->getResolution ();
      color_bit_depth  = PointCloudEncoder->color_coder_.getBitDepth ();
      point_resolution= PointCloudEncoder->point_coder_.getPrecision ();
      //PointCloudEncoder->getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);
      min_x = -250;
      min_y = -250;
      min_z = -250;
      max_x = 250;
      max_y = 250;
      max_z = 250;


      // encode amount of points
      if (PointCloudEncoder->do_voxel_grid_enDecoding_)
        PointCloudEncoder->point_count_ = PointCloudEncoder->getLeafCount(); // LEAF COUNT HW ??
      else
        PointCloudEncoder->point_count_ = PointCloudEncoder->object_count_; // Point cloud points

      // encode coding configuration
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder->do_voxel_grid_enDecoding_), sizeof (PointCloudEncoder->do_voxel_grid_enDecoding_));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&cloud_with_color_), sizeof (cloud_with_color_));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder->point_count_), sizeof (PointCloudEncoder->point_count_));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&octree_resolution), sizeof (octree_resolution));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&color_bit_depth), sizeof (color_bit_depth));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_resolution), sizeof (point_resolution));

      // encode octree bounding box
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x), sizeof (min_x));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y), sizeof (min_y));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z), sizeof (min_z));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x), sizeof (max_x));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y), sizeof (max_y));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z), sizeof (max_z));
    }


}









void Alfa_Pc_Compress::divide_in_octants_test(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud){ // Funciona. mais rapido que a pcl

     pcl::PointXYZRGB minPt, maxPt;
     minPt.x = 0;
     minPt.y = 0;
     minPt.z = 0;
     maxPt.x = 0;
     maxPt.y = 0;
     maxPt.z = 0;

     for(auto &point: *input_cloud){
         if(point.x <= minPt.x)
                minPt.x = point.x;
         if(point.y <= minPt.y)
                minPt.y = point.y;
         if(point.z <= minPt.z)
                minPt.z = point.z;
         if(point.x >= maxPt.x)
                maxPt.x = point.x;
         if(point.y >= maxPt.y)
                maxPt.y = point.y;
         if(point.z >= maxPt.z)
                maxPt.z = point.z;
     }




     mid_x = (maxPt.x + minPt.x)/2 ;
     mid_y = (maxPt.y + minPt.y)/2 ;
     mid_z = (maxPt.z + minPt.z)/2 ;

     printf("%f \n",maxPt.x);
     printf("%f \n",maxPt.y);
     printf("%f \n",maxPt.z);
     printf("%f \n",minPt.x);
     printf("%f \n",minPt.y);
     printf("%f \n",minPt.z);

     printf("%f \n",mid_x);
     printf("%f \n",mid_y);
     printf("%f \n",mid_z);
     printf("%d \n",input_cloud->size());


     // for more exe. time savings, try multithreading in division

     for (int i=0;i<input_cloud->size();i++) {
         pcl::PointXYZRGB point = (*input_cloud)[i];

         if(point.x <= mid_x){
             if(point.y <= mid_y){
                 if(point.z <= mid_z)
                     octant_0->push_back(point);
                 else
                     octant_1->push_back(point);
             }
             else {
                 if(point.z <= mid_z)
                     octant_2->push_back(point);
                 else
                     octant_3->push_back(point);
             }
         }
         else {
             if(point.y <= mid_y){
                 if(point.z <= mid_z)
                     octant_4->push_back(point);
                 else
                     octant_5->push_back(point);
             }
             else{
                 if(point.z <= mid_z)
                     octant_6->push_back(point);
                 else
                     octant_7->push_back(point);
             }
         }


     }

}

void Alfa_Pc_Compress::divide_in_octants(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) // Bad Division ?????
{
    //pcl::PointXYZRGB minPt, maxPt;
    //pcl::getMinMax3D(*input_cloud,minPt,maxPt);


    octant_0->header = input_cloud->header;
    octant_1->header = input_cloud->header;
    octant_2->header = input_cloud->header;
    octant_3->header = input_cloud->header;
    octant_4->header = input_cloud->header;
    octant_5->header = input_cloud->header;
    octant_6->header = input_cloud->header;
    octant_7->header = input_cloud->header;




    for (int i=0;i<input_cloud->size();i++) {
        pcl::PointXYZRGB point = (*input_cloud)[i];

        if(point.x >= 0 && point.y >=0 && point.z >=0)
                octant_0->push_back(point);
        else if(point.x < 0 && point.y >= 0 && point.z >= 0)
                octant_1->push_back(point);
        else if(point.x < 0 && point.y < 0 && point.z >= 0)
                octant_2->push_back(point);
        else if(point.x >= 0 && point.y < 0 && point.z >= 0)
                octant_3->push_back(point);
        else if(point.x >= 0 && point.y >= 0 && point.z < 0)
                octant_4->push_back(point);
        else if(point.x < 0 && point.y >= 0 && point.z < 0)
                octant_5->push_back(point);
        else if(point.x < 0 && point.y < 0 && point.z < 0)
                octant_6->push_back(point);
        else if(point.x >= 0 && point.y < 0 && point.z < 0)
                octant_7->push_back(point);

    }
}




void Alfa_Pc_Compress::divide_in_octants_multi(int thread_number,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
    for(int i =(input_cloud->size()/number_threads)*thread_number; i<= (input_cloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZRGB point = (*input_cloud)[i];

        if(point.x >= 0 && point.y >=0 && point.z >=0){
                mux_oct_0.lock();
                octant_0->push_back(point);
                mux_oct_0.unlock();
        }
        else if(point.x < 0 && point.y >= 0 && point.z >= 0){
                mux_oct_1.lock();
                octant_1->push_back(point);
                mux_oct_1.unlock();
        }
        else if(point.x < 0 && point.y < 0 && point.z >= 0){
                mux_oct_2.lock();
                octant_2->push_back(point);
                mux_oct_2.unlock();
        }
        else if(point.x >= 0 && point.y < 0 && point.z >= 0){
                mux_oct_3.lock();
                octant_3->push_back(point);
                mux_oct_3.unlock();
        }
        else if(point.x >= 0 && point.y >= 0 && point.z < 0){
                mux_oct_4.lock();
                octant_4->push_back(point);
                mux_oct_4.unlock();
        }
        else if(point.x < 0 && point.y >= 0 && point.z < 0){
                mux_oct_5.lock();
                octant_5->push_back(point);
                mux_oct_5.unlock();
        }
        else if(point.x < 0 && point.y < 0 && point.z < 0){
                mux_oct_6.lock();
                octant_6->push_back(point);
                mux_oct_6.unlock();
        }
        else if(point.x >= 0 && point.y < 0 && point.z < 0){
                mux_oct_7.lock();
                octant_7->push_back(point);
                mux_oct_7.unlock();
        }

    }
}

void Alfa_Pc_Compress::compress_octant(pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_encoder,pcl::PointCloud<pcl::PointXYZRGB>::Ptr octant)
{
    std::cout << "Octant Size: " << octant->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (point_cloud_encoder->getTreeDepth ());


    point_cloud_encoder->setInputCloud(octant);

    point_cloud_encoder->addPointsFromInputCloud();


    if( point_cloud_encoder->getLeafCount()>0){
         point_cloud_encoder->cloud_with_color_ = false;
         point_cloud_encoder->cloud_with_color_ &= point_cloud_encoder->do_color_encoding_;

         point_cloud_encoder->i_frame_ |= (recent_tree_depth !=   point_cloud_encoder->getTreeDepth ());

        if ( point_cloud_encoder->i_frame_counter_++== point_cloud_encoder->i_frame_rate_)
        {
             point_cloud_encoder->i_frame_counter_ =0;
             point_cloud_encoder->i_frame_ = true;
        }

         point_cloud_encoder->frame_ID_++;

        std::cout << "Thread 0 Frame ID :\n" << point_cloud_encoder->frame_ID_ <<std::endl;

        if (! point_cloud_encoder->do_voxel_grid_enDecoding_)
        {
             point_cloud_encoder->point_count_data_vector_.clear ();
             point_cloud_encoder->point_count_data_vector_.reserve (octant->points.size ());
        }

        // initialize color encoding
         point_cloud_encoder->color_coder_.initializeEncoding ();
         point_cloud_encoder->color_coder_.setPointCount (static_cast<unsigned int> (octant->points.size ()));
         point_cloud_encoder->color_coder_.setVoxelCount (static_cast<unsigned int> ( point_cloud_encoder->getLeafCount()));

        // initialize point encoding
         point_cloud_encoder->point_coder_.initializeEncoding ();
         point_cloud_encoder->point_coder_.setPointCount (static_cast<unsigned int> (octant->points.size ()));


        if( point_cloud_encoder->i_frame_){
             point_cloud_encoder->serializeTree( point_cloud_encoder->binary_tree_data_vector_,false);
        }else{
             point_cloud_encoder->serializeTree( point_cloud_encoder->binary_tree_data_vector_,true);
        }



    }

}

void Alfa_Pc_Compress::print_statistics_octant(pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_encoder)
{
    if (point_cloud_encoder->b_show_statistics_)
    {
        float bytes_per_XYZ = static_cast<float> (point_cloud_encoder->compressed_point_data_len_) / static_cast<float> (point_cloud_encoder->point_count_);
        float bytes_per_color = static_cast<float> (point_cloud_encoder->compressed_color_data_len_) / static_cast<float> (point_cloud_encoder->point_count_);
        PCL_INFO ("*** POINTCLOUD ENCODING *** 2\n");
        PCL_INFO ("Frame ID: %d\n", point_cloud_encoder->frame_ID_);
        if (point_cloud_encoder->i_frame_)
            PCL_INFO ("Encoding Frame: Intra frame\n");
        else
            PCL_INFO ("Encoding Frame: Prediction frame\n");
        PCL_INFO ("Number of encoded points: -------------------------%ld\n", point_cloud_encoder->point_count_);
        PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
        PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
        PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
        PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_cloud_encoder->point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
        PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (point_cloud_encoder->compressed_point_data_len_ + point_cloud_encoder->compressed_color_data_len_) / 1024.0f);
        PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ + bytes_per_color);
        PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
    }
}






void Alfa_Pc_Compress::run_worker_thread_octants_full(uint8_t thread_number, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{

    if(thread_number==0){

        //std::stringstream compressed_data;
        //PointCloudEncoder_0->encodePointCloud_2(octant_0,compressed_data);

        compress_octant_0();
        double min_x_0,min_y_0,min_z_0,max_x_0,max_y_0,max_z_0;
        PointCloudEncoder_0->getBoundingBox(min_x_0, min_y_0, min_z_0, max_x_0, max_y_0, max_z_0);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_0->writeFrameHeader(compressed_data);
        PointCloudEncoder_0->entropyEncoding(compressed_data);
        PointCloudEncoder_0->switchBuffers();
        PointCloudEncoder_0->object_count_ = 0;
        octant_0->clear();
        //print_statistics_octant(PointCloudEncoder_0);
        //print_statistics_octant(thread_number);

    }
    if(thread_number==1){
//        std::stringstream compressed_data;
//        PointCloudEncoder_1->encodePointCloud_2(octant_1,compressed_data);
        compress_octant_1();
        double min_x_1,min_y_1,min_z_1,max_x_1,max_y_1,max_z_1;
        PointCloudEncoder_1->getBoundingBox(min_x_1, min_y_1, min_z_1, max_x_1, max_y_1, max_z_1);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_1->writeFrameHeader(compressed_data);
        PointCloudEncoder_1->entropyEncoding(compressed_data);
        PointCloudEncoder_1->switchBuffers();
        PointCloudEncoder_1->object_count_ = 0;
        octant_1->clear();
        //print_statistics_octant(PointCloudEncoder_1);

    }
    if(thread_number==2){
        //std::stringstream compressed_data;
        //PointCloudEncoder_2->encodePointCloud_2(octant_2,compressed_data);
        compress_octant_2();
        double min_x_2,min_y_2,min_z_2,max_x_2,max_y_2,max_z_2;
        PointCloudEncoder_2->getBoundingBox(min_x_2, min_y_2, min_z_2, max_x_2, max_y_2, max_z_2);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_2->writeFrameHeader(compressed_data);
        PointCloudEncoder_2->entropyEncoding(compressed_data);
        PointCloudEncoder_2->switchBuffers();
        PointCloudEncoder_2->object_count_ = 0;
        octant_2->clear();
        //print_statistics_octant(PointCloudEncoder_2);

    }
    if(thread_number==3){
        //std::stringstream compressed_data;
        //PointCloudEncoder_3->encodePointCloud_2(octant_3,compressed_data);
        compress_octant_3();
        double min_x_3,min_y_3,min_z_3,max_x_3,max_y_3,max_z_3;
        PointCloudEncoder_3->getBoundingBox(min_x_3, min_y_3, min_z_3, max_x_3, max_y_3, max_z_3);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_3->writeFrameHeader(compressed_data);
        PointCloudEncoder_3->entropyEncoding(compressed_data);
        PointCloudEncoder_3->switchBuffers();
        PointCloudEncoder_3->object_count_ = 0;
        octant_3->clear();
        //print_statistics_octant(PointCloudEncoder_3);

    }
    if(thread_number==4){
        //std::stringstream compressed_data;
        //PointCloudEncoder_4->encodePointCloud_2(octant_4,compressed_data);
        compress_octant_4();
        double min_x_4,min_y_4,min_z_4,max_x_4,max_y_4,max_z_4;
        PointCloudEncoder_4->getBoundingBox(min_x_4, min_y_4, min_z_4, max_x_4, max_y_4, max_z_4);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_4->writeFrameHeader(compressed_data);
        PointCloudEncoder_4->entropyEncoding(compressed_data);
        PointCloudEncoder_4->switchBuffers();
        PointCloudEncoder_4->object_count_ = 0;
        octant_4->clear();
        //print_statistics_octant(PointCloudEncoder_4);
    }
    if(thread_number==5){
        //std::stringstream compressed_data;
        //PointCloudEncoder_5->encodePointCloud_2(octant_5,compressed_data);
        compress_octant_5();
        double min_x_5,min_y_5,min_z_5,max_x_5,max_y_5,max_z_5;
        PointCloudEncoder_5->getBoundingBox(min_x_5, min_y_5, min_z_5, max_x_5, max_y_5, max_z_5);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_5->writeFrameHeader(compressed_data);
        PointCloudEncoder_5->entropyEncoding(compressed_data);
        PointCloudEncoder_5->switchBuffers();
        PointCloudEncoder_5->object_count_ = 0;
        octant_5->clear();
        //print_statistics_octant(PointCloudEncoder_5);

    }
    if(thread_number==6){
        //std::stringstream compressed_data;
        //PointCloudEncoder_6->encodePointCloud_2(octant_6,compressed_data);
        compress_octant_6();
        double min_x_6,min_y_6,min_z_6,max_x_6,max_y_6,max_z_6;
        PointCloudEncoder_6->getBoundingBox(min_x_6, min_y_6, min_z_6, max_x_6, max_y_6, max_z_6);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_6->writeFrameHeader(compressed_data);
        PointCloudEncoder_6->entropyEncoding(compressed_data);
        PointCloudEncoder_6->switchBuffers();
        PointCloudEncoder_6->object_count_ = 0;
        octant_6->clear();
        //print_statistics_octant(PointCloudEncoder_6);

    }
    if(thread_number==7){
//        std::stringstream compressed_data;
//        PointCloudEncoder_7->encodePointCloud_2(octant_7,compressed_data);
        compress_octant_7();
        double min_x_7,min_y_7,min_z_7,max_x_7,max_y_7,max_z_7;
        PointCloudEncoder_7->getBoundingBox(min_x_7, min_y_7, min_z_7, max_x_7, max_y_7, max_z_7);
        //test all-in multithreading
        std::stringstream compressed_data;
        PointCloudEncoder_7->writeFrameHeader(compressed_data);
        PointCloudEncoder_7->entropyEncoding(compressed_data);
        PointCloudEncoder_7->switchBuffers();
        PointCloudEncoder_7->object_count_ = 0;
        //print_statistics_octant(PointCloudEncoder_7);
        octant_7->clear();
    }



}




void Alfa_Pc_Compress::run_worker_thread_octants(uint8_t thread_number, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{

    if(thread_number==0){  
        compress_octant_0();
        octant_0->clear();
        std::cout << "Acabei Thread " << (int)thread_number << "\n";
        oct_0 = true;

    }
    if(thread_number==1){

        compress_octant_1();
        octant_1->clear();
        std::cout << "Acabei Thread " << (int)thread_number << "\n";
        oct_1 = true;

    }
    if(thread_number==2){

        compress_octant_2();
        octant_2->clear();
        oct_2 = true;
        std::cout << "Acabei Thread " << (int)thread_number << "\n";

    }
    if(thread_number==3){

        compress_octant_3();
        octant_3->clear();
        oct_3 = true;
        std::cout << "Acabei Thread " << (int)thread_number << "\n";

    }
    if(thread_number==4){

        compress_octant_4();
        octant_4->clear();
        oct_4 = true;
        std::cout << "Acabei Thread " << (int)thread_number << "\n";

    }
    if(thread_number==5){

        compress_octant_5();
        octant_5->clear();
        oct_5 = true;
        std::cout << "Acabei Thread " << (int)thread_number << "\n";

    }
    if(thread_number==6){

        compress_octant_6();
        octant_6->clear();
        oct_6 = true;
        std::cout << "Acabei Thread " << (int)thread_number << "\n";


    }
    if(thread_number==7){

        compress_octant_7();
        octant_7->clear();
        oct_7 = true;
        std::cout << "Acabei Thread " << (int)thread_number << "\n";
    }



}




void Alfa_Pc_Compress::my_write_frame_header(ostream &compressed_tree_data_out_arg)
{
    compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (frame_header_identifier), strlen (frame_header_identifier));
    // encode point cloud header id
    compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder_0->frame_ID_), sizeof (PointCloudEncoder_0->frame_ID_));
    // encode frame type (I/P-frame)
    compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder_0->i_frame_), sizeof (PointCloudEncoder_0->i_frame_));


    if(PointCloudEncoder_0->i_frame_){
    // teste stats of encoders //


        double min_x, min_y, min_z, max_x, max_y, max_z;
        double octree_resolution;
        unsigned char color_bit_depth;
        double point_resolution;


        // ****************** Fazer esta parte em multithreading ??? *************
        double min_x_0, min_y_0, min_z_0, max_x_0, max_y_0, max_z_0;
        double min_x_1, min_y_1, min_z_1, max_x_1, max_y_1, max_z_1;
        double min_x_2, min_y_2, min_z_2, max_x_2, max_y_2, max_z_2;
        double min_x_3, min_y_3, min_z_3, max_x_3, max_y_3, max_z_3;
        double min_x_4, min_y_4, min_z_4, max_x_4, max_y_4, max_z_4;
        double min_x_5, min_y_5, min_z_5, max_x_5, max_y_5, max_z_5;
        double min_x_6, min_y_6, min_z_6, max_x_6, max_y_6, max_z_6;
        double min_x_7, min_y_7, min_z_7, max_x_7, max_y_7, max_z_7;


        unsigned int size_0,size_1,size_2,size_3,size_4,size_5,size_6,size_7;

        octree_resolution = PointCloudEncoder_0->getResolution ();
        color_bit_depth  = PointCloudEncoder_0->color_coder_.getBitDepth ();
        point_resolution = PointCloudEncoder_0->point_coder_.getPrecision ();


        PointCloudEncoder_0->getBoundingBox(min_x_0, min_y_0, min_z_0, max_x_0, max_y_0, max_z_0);
        PointCloudEncoder_1->getBoundingBox(min_x_1, min_y_1, min_z_1, max_x_1, max_y_1, max_z_1);
        PointCloudEncoder_2->getBoundingBox(min_x_2, min_y_2, min_z_2, max_x_2, max_y_2, max_z_2);
        PointCloudEncoder_3->getBoundingBox(min_x_3, min_y_3, min_z_3, max_x_3, max_y_3, max_z_3);
        PointCloudEncoder_4->getBoundingBox(min_x_4, min_y_4, min_z_4, max_x_4, max_y_4, max_z_4);
        PointCloudEncoder_5->getBoundingBox(min_x_5, min_y_5, min_z_5, max_x_5, max_y_5, max_z_5);
        PointCloudEncoder_7->getBoundingBox(min_x_7, min_y_7, min_z_7, max_x_7, max_y_7, max_z_7);
        PointCloudEncoder_6->getBoundingBox(min_x_6, min_y_6, min_z_6, max_x_6, max_y_6, max_z_6);



       /* printf("\n-----Cluster 0 -----\n");

        printf("min_x: %f\n",min_x_0);
        printf("min_y: %f\n",min_y_0);
        printf("min_z: %f\n",min_z_0);
        printf("max_x: %f\n",max_x_0);
        printf("max_y: %f\n",max_y_0);
        printf("max_z: %f\n",max_z_0);

        printf("-----Cluster 1 -----\n");

        printf("min_x: %f\n",min_x_1);
        printf("min_y: %f\n",min_y_1);
        printf("min_z: %f\n",min_z_1);
        printf("max_x: %f\n",max_x_1);
        printf("max_y: %f\n",max_y_1);
        printf("max_z: %f\n",max_z_1);


        printf("min_x: %f\n",min_x);
        printf("min_y: %f\n",min_y);
        printf("min_z: %f\n",min_z);
        printf("max_x: %f\n",max_x);
        printf("max_y: %f\n",max_y);
        printf("max_z: %f\n",max_z);*/


        if(PointCloudEncoder_0->do_voxel_grid_enDecoding_){
            point_count_0 = PointCloudEncoder_0->getLeafCount();
            point_count_1 = PointCloudEncoder_1->getLeafCount();
            point_count_2 = PointCloudEncoder_2->getLeafCount();
            point_count_3 = PointCloudEncoder_3->getLeafCount();
            point_count_4 = PointCloudEncoder_4->getLeafCount();
            point_count_5 = PointCloudEncoder_5->getLeafCount();
            point_count_6 = PointCloudEncoder_6->getLeafCount();
            point_count_7 = PointCloudEncoder_7->getLeafCount();

        }
        else{
            point_count_0 = PointCloudEncoder_0->object_count_;
            point_count_1 = PointCloudEncoder_1->object_count_;
            point_count_2 = PointCloudEncoder_2->object_count_;
            point_count_3 = PointCloudEncoder_3->object_count_;
            point_count_4 = PointCloudEncoder_4->object_count_;
            point_count_5 = PointCloudEncoder_5->object_count_;
            point_count_6 = PointCloudEncoder_6->object_count_;
            point_count_7 = PointCloudEncoder_7->object_count_;
        }


        //************** ATÉ AQUI EM MULTI ????? ******************************



        // encode coding configuration
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder_0->do_voxel_grid_enDecoding_), sizeof (PointCloudEncoder_0->do_voxel_grid_enDecoding_));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&PointCloudEncoder_0->cloud_with_color_), sizeof (PointCloudEncoder_0->cloud_with_color_));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_0), sizeof (point_count_0));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&octree_resolution), sizeof (octree_resolution));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&color_bit_depth), sizeof (color_bit_depth));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_resolution), sizeof (point_resolution));

        // encode octree bounding box octant_0
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_0), sizeof (min_x_0));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_0), sizeof (min_y_0));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_0), sizeof (min_z_0));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_0), sizeof (max_x_0));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_0), sizeof (max_y_0));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_0), sizeof (max_z_0));


        // encode octree bounding box octant_1
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_1), sizeof (point_count_1));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_1), sizeof (min_x_1));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_1), sizeof (min_y_1));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_1), sizeof (min_z_1));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_1), sizeof (max_x_1));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_1), sizeof (max_y_1));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_1), sizeof (max_z_1));


        // encode octree bounding box octant_2
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_2), sizeof (point_count_2));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_2), sizeof (min_x_2));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_2), sizeof (min_y_2));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_2), sizeof (min_z_2));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_2), sizeof (max_x_2));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_2), sizeof (max_y_2));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_2), sizeof (max_z_2));


        // encode octree bounding box octant_3
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_3), sizeof (point_count_3));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_3), sizeof (min_x_3));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_3), sizeof (min_y_3));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_3), sizeof (min_z_3));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_3), sizeof (max_x_3));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_3), sizeof (max_y_3));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_3), sizeof (max_z_3));


        // encode octree bounding box octant_4
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_4), sizeof (point_count_4));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_4), sizeof (min_x_4));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_4), sizeof (min_y_4));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_4), sizeof (min_z_4));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_4), sizeof (max_x_4));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_4), sizeof (max_y_4));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_4), sizeof (max_z_4));


        // encode octree bounding box octant_5
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_5), sizeof (point_count_5));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_5), sizeof (min_x_5));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_5), sizeof (min_y_5));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_5), sizeof (min_z_5));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_5), sizeof (max_x_5));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_5), sizeof (max_y_5));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_5), sizeof (max_z_5));


        // encode octree bounding box octant_6
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_6), sizeof (point_count_6));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_6), sizeof (min_x_6));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_6), sizeof (min_y_6));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_6), sizeof (min_z_6));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_6), sizeof (max_x_6));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_6), sizeof (max_y_6));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_6), sizeof (max_z_6));


        // encode octree bounding box octant_7
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_7), sizeof (point_count_7));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x_7), sizeof (min_x_7));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y_7), sizeof (min_y_7));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z_7), sizeof (min_z_7));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x_7), sizeof (max_x_7));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y_7), sizeof (max_y_7));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z_7), sizeof (max_z_7));



        size_0 = PointCloudEncoder_0->binary_tree_data_vector_.size();
        size_1 = PointCloudEncoder_1->binary_tree_data_vector_.size();
        size_2 = PointCloudEncoder_2->binary_tree_data_vector_.size();
        size_3 = PointCloudEncoder_3->binary_tree_data_vector_.size();
        size_4 = PointCloudEncoder_4->binary_tree_data_vector_.size();
        size_5 = PointCloudEncoder_5->binary_tree_data_vector_.size();
        size_6 = PointCloudEncoder_6->binary_tree_data_vector_.size();
        size_7 = PointCloudEncoder_7->binary_tree_data_vector_.size();

        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_0), sizeof (size_0));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_1), sizeof (size_1));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_2), sizeof (size_2));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_3), sizeof (size_3));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_4), sizeof (size_4));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_5), sizeof (size_5));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_6), sizeof (size_6));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&size_7), sizeof (size_7));


    }

}





void Alfa_Pc_Compress::store_occupancy_code_hardware(std::vector<char> vec, u64 *pointer)
{
    int index = 0;
    int8_t occupancy_code[8];

    for(index = 0;index<vec.size();index++)
    {
        occupancy_code[0] = vec[0 + 8*index];
        occupancy_code[1] = vec[1 + 8*index];
        occupancy_code[2] = vec[2 + 8*index];
        occupancy_code[3] = vec[3 + 8*index];
        occupancy_code[4] = vec[4 + 8*index];
        occupancy_code[5] = vec[5 + 8*index];
        occupancy_code[6] = vec[6 + 8*index];
        occupancy_code[7] = vec[7 + 8*index];
        memcpy((void*)(pointer+index),occupancy_code,sizeof(int8_t)*8);
    }

}







alfa_msg::AlfaConfigure::Response Alfa_Pc_Compress::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    update_compressionSettings(req);
    alfa_msg::AlfaConfigure::Response response;
    response.return_status = 1;
    return response;

}




void Alfa_Pc_Compress::metrics(std::stringstream& compressed_data, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud, double duration)
{
    compressed_data.seekg(0,ios::end);
    size_compressed = compressed_data.tellg();


    size_original = (static_cast<float> (output_cloud->points.size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;


    ROS_INFO("Tree depth: %d\n",PointCloudEncoder->getTreeDepth());



    // alfa metrics
    alfa_msg::MetricMessage new_message;

    new_message.metric = size_original/1000;
    new_message.units = "kB";
    new_message.metric_name = "Original Size";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = output_cloud->size();
    new_message.units = "";
    new_message.metric_name = "Nº of Points in Point Cloud";
    output_metrics.metrics.push_back(new_message);


    new_message.metric = duration;
    new_message.units = "ms";
    new_message.metric_name = "Total processing time";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = size_compressed/1000;
    new_message.units = "kB";
    new_message.metric_name = "Compressed Size";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = size_original/size_compressed;
    new_message.units = "";
    new_message.metric_name = "Compression Ratio";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = PointCloudEncoder->getTreeDepth();
    new_message.units = "";
    new_message.metric_name = "Octree Depth";
    output_metrics.metrics.push_back(new_message);



}

void Alfa_Pc_Compress::update_compressionSettings(const alfa_msg::AlfaConfigure::Request configs)
{
    //compression_profile_.octreeResolution = configs.configurations[0].config;


    multi_thread = configs.configurations[1].config;
    //delete(PointCloudEncoder);
    size_test = configs.configurations[2].config;
    depth_test = configs.configurations[3].config;
    if(multi_thread){
        set_compression_profile();
        PointCloudEncoder_0->setResolution(configs.configurations[0].config);
        PointCloudEncoder_1->setResolution(configs.configurations[0].config);
        PointCloudEncoder_2->setResolution(configs.configurations[0].config);
        PointCloudEncoder_3->setResolution(configs.configurations[0].config);
        PointCloudEncoder_4->setResolution(configs.configurations[0].config);
        PointCloudEncoder_5->setResolution(configs.configurations[0].config);
        PointCloudEncoder_6->setResolution(configs.configurations[0].config);
        PointCloudEncoder_7->setResolution(configs.configurations[0].config);
    }else{
        set_compression_profile();
        PointCloudEncoder->setResolution(configs.configurations[0].config);

    }

}




void Alfa_Pc_Compress::exe_time()
{
    tempos_test = tempos_test/100 ;
    size_compressed_test = size_compressed_test/100;
    size_original_test = (size_original_test)/100;
    points_second = points_second/100;
    //std::ofstream myFile("./output/exe_time");
    //myFile<< "Exe. Time: "<< tempos_test << std::endl << "Point Cloud Size: "<< size_original_test << std::endl << "Compressed Size: "<<size_compressed_test<< std::endl << "Ratio: " << size_original_test/size_compressed_test << std::endl ;
    //myFile.close();
    ROS_INFO("-----------Acabei------------------------------------------------------------- \n");
    ROS_INFO("Time: %d\n", tempos_test);
    ROS_INFO("Point Cloud Size: %f\n", size_original_test);
    ROS_INFO("Compressed Size: %f\n", size_compressed_test);
    ROS_INFO("Ratio: %f\n", size_original_test/size_compressed_test);
    ROS_INFO("Points/s: %f\n", points_second);

    x=0;
    size_compressed_test = 0;
    size_original_test = 0;
    points_second = 0;
}


//void Alfa_Pc_Compress::do_Compression(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
//{

//    using namespace std::chrono;

//    std::stringstream compressed_data_;

//    ROS_INFO("Compressing cloud with frame [%s]", in_cloud->header.frame_id.c_str());
//    this->in_cloud = in_cloud;

//    auto start = high_resolution_clock::now();

//    //********* compress point cloud ************ //
//    //point_cloud_encoder->setInputCloud(in_cloud);


//    point_cloud_encoder->encodePointCloud(in_cloud,compressed_data_);



//    auto stop = high_resolution_clock::now();
//    auto duration = duration_cast<milliseconds>(stop - start);
//    ROS_INFO("Compressing in %ld ms",duration.count());
//    ROS_INFO("Tree depth: %d\n",point_cloud_encoder->getTreeDepth());

//    // testes tempos
//    tempos = tempos + duration.count();
//    compressed_data_.seekg(0,ios::end);
//    size_compressed = size_compressed + compressed_data_.tellg();
//    size_original = size_original + static_cast<float> (in_cloud->size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f;
//    x++;
//    //test
//    if(x==100){
//      exe_time();
//    }
//}



void Alfa_Pc_Compress::run_worker(int thread_number)
{
    std::cout << "THREAD: " << thread_number << std::endl;
    //std::cout << "Cloud Size: " << pcloud->size()<< std::endl;

    for(int i = (in_cloud->size()/number_threads)*thread_number;i<= (in_cloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZRGB point = (*in_cloud)[i];

        //push point to cluster
        if(thread_number == 0)
        {
            mutex_cluster.lock();
            cluster0->push_back(point);
            mutex_cluster.unlock();


        }else if (thread_number == 1)
        {
            mutex_cluster.lock();
            cluster1->push_back(point);
            mutex_cluster.unlock();
        }


    }


   // vai ter de ser array de encoders
    //PointCloudEncoder->encodePointCloud(clusters[thread_number],compressed_data_vector[thread_number]);
    std::cout << "Fiz clusters " << std::endl;
    std::cout << "Cluster 0 Size: " << cluster0->size()<< std::endl;
    std::cout << "Cluster 1 Size: " << cluster1->size()<< std::endl;
    //mutex_encoder.lock();
    //encoder_vector[thread_number]->encodePointCloud_2(cluster_vector[thread_number],compressed_data_vector[thread_number]);
    //mutex_encoder.unlock();

    //mutex_compressed_data.lock();
    //output_compressed.data += compressed_data_vector[thread_number].str();
    //mutex_compressed_data.unlock();

    if(thread_number == 0)
    {
        std::stringstream compressed_data;
        //cluster1->header =
        PointCloudEncoder->encodePointCloud_2(cluster0,compressed_data);
        std::cout << "Fiz compressao encoder 0" << std::endl;

        mutex_compressed_data1.lock();
        output_compressed.data += compressed_data.str();
        mutex_compressed_data1.unlock();

    }
    if(thread_number == 1)
    {
        std::stringstream compressed_data1;
        PointCloudEncoder1->encodePointCloud_2(cluster1,compressed_data1);
        std::cout << "Fiz compressao encoder 1" << std::endl;

        mutex_compressed_data2.lock();
        output_compressed.data += compressed_data1.str();
        mutex_compressed_data2.unlock();

    }


}

void Alfa_Pc_Compress::compress_octant_0()
{
    std::cout << "Octant 0 Size: " << octant_0->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_0->getTreeDepth ());


    PointCloudEncoder_0->setInputCloud(octant_0);

    mux_oct_0.lock();
    PointCloudEncoder_0->addPointsFromInputCloud();
    mux_oct_0.unlock();

    if( PointCloudEncoder_0->getLeafCount()>0){
        PointCloudEncoder_0->cloud_with_color_ = false;
        PointCloudEncoder_0->cloud_with_color_ &= PointCloudEncoder_0->do_color_encoding_;

        PointCloudEncoder_0->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_0->getTreeDepth ());

        if ( PointCloudEncoder_0->i_frame_counter_++== PointCloudEncoder_0->i_frame_rate_)
        {
            PointCloudEncoder_0->i_frame_counter_ =0;
            PointCloudEncoder_0->i_frame_ = true;
        }

        PointCloudEncoder_0->frame_ID_++;

        std::cout << "Thread 0 Frame ID :" << PointCloudEncoder_0->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_0->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_0->point_count_data_vector_.clear ();
            PointCloudEncoder_0->point_count_data_vector_.reserve (octant_0->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_0->color_coder_.initializeEncoding ();
        PointCloudEncoder_0->color_coder_.setPointCount (static_cast<unsigned int> (octant_0->points.size ()));
        PointCloudEncoder_0->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_0->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_0->point_coder_.initializeEncoding ();
        PointCloudEncoder_0->point_coder_.setPointCount (static_cast<unsigned int> (octant_0->points.size ()));


        if( PointCloudEncoder_0->i_frame_){
            PointCloudEncoder_0->serializeTree( PointCloudEncoder_0->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_0->serializeTree( PointCloudEncoder_0->binary_tree_data_vector_,true);
        }



    }
}

void Alfa_Pc_Compress::compress_octant_1()
{
    std::cout << "Octant 1 Size: " << octant_1->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_1->getTreeDepth ());


    PointCloudEncoder_1->setInputCloud(octant_1);

    mux_oct_1.lock();
    PointCloudEncoder_1->addPointsFromInputCloud();
    mux_oct_1.unlock();

    if( PointCloudEncoder_1->getLeafCount()>0){
        PointCloudEncoder_1->cloud_with_color_ = false;
        PointCloudEncoder_1->cloud_with_color_ &= PointCloudEncoder_1->do_color_encoding_;

        PointCloudEncoder_1->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_1->getTreeDepth ());

        if ( PointCloudEncoder_1->i_frame_counter_++== PointCloudEncoder_1->i_frame_rate_)
        {
            PointCloudEncoder_1->i_frame_counter_ =0;
            PointCloudEncoder_1->i_frame_ = true;
        }

        PointCloudEncoder_1->frame_ID_++;

        std::cout << "Thread 1 Frame ID :" << PointCloudEncoder_1->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_1->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_1->point_count_data_vector_.clear ();
            PointCloudEncoder_1->point_count_data_vector_.reserve (octant_1->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_1->color_coder_.initializeEncoding ();
        PointCloudEncoder_1->color_coder_.setPointCount (static_cast<unsigned int> (octant_1->points.size ()));
        PointCloudEncoder_1->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_1->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_1->point_coder_.initializeEncoding ();
        PointCloudEncoder_1->point_coder_.setPointCount (static_cast<unsigned int> (octant_1->points.size ()));


        if( PointCloudEncoder_1->i_frame_){
            PointCloudEncoder_1->serializeTree( PointCloudEncoder_1->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_1->serializeTree( PointCloudEncoder_1->binary_tree_data_vector_,true);
        }



    }
}


void Alfa_Pc_Compress::compress_octant_2()
{
    std::cout << "Octant 2 Size: " << octant_2->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_2->getTreeDepth ());


    PointCloudEncoder_2->setInputCloud(octant_2);
    mux_oct_2.lock();
    PointCloudEncoder_2->addPointsFromInputCloud();
    mux_oct_2.unlock();

    if( PointCloudEncoder_2->getLeafCount()>0){
        PointCloudEncoder_2->cloud_with_color_ = false;
        PointCloudEncoder_2->cloud_with_color_ &= PointCloudEncoder_2->do_color_encoding_;

        PointCloudEncoder_2->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_2->getTreeDepth ());

        if ( PointCloudEncoder_2->i_frame_counter_++== PointCloudEncoder_2->i_frame_rate_)
        {
            PointCloudEncoder_2->i_frame_counter_ =0;
            PointCloudEncoder_2->i_frame_ = true;
        }

        PointCloudEncoder_2->frame_ID_++;

        std::cout << "Thread 2 Frame ID :" << PointCloudEncoder_2->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_2->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_2->point_count_data_vector_.clear ();
            PointCloudEncoder_2->point_count_data_vector_.reserve (octant_2->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_2->color_coder_.initializeEncoding ();
        PointCloudEncoder_2->color_coder_.setPointCount (static_cast<unsigned int> (octant_2->points.size ()));
        PointCloudEncoder_2->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_2->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_2->point_coder_.initializeEncoding ();
        PointCloudEncoder_2->point_coder_.setPointCount (static_cast<unsigned int> (octant_2->points.size ()));


        if( PointCloudEncoder_2->i_frame_){
            PointCloudEncoder_2->serializeTree( PointCloudEncoder_2->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_2->serializeTree( PointCloudEncoder_2->binary_tree_data_vector_,true);
        }



    }
}

void Alfa_Pc_Compress::compress_octant_3()
{
    std::cout << "Octant 3 Size: " << octant_3->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_3->getTreeDepth ());


    PointCloudEncoder_3->setInputCloud(octant_3);

    mux_oct_3.lock();
    PointCloudEncoder_3->addPointsFromInputCloud();
    mux_oct_3.unlock();

    if( PointCloudEncoder_3->getLeafCount()>0){
        PointCloudEncoder_3->cloud_with_color_ = false;
        PointCloudEncoder_3->cloud_with_color_ &= PointCloudEncoder_3->do_color_encoding_;

        PointCloudEncoder_3->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_3->getTreeDepth ());

        if ( PointCloudEncoder_3->i_frame_counter_++== PointCloudEncoder_3->i_frame_rate_)
        {
            PointCloudEncoder_3->i_frame_counter_ =0;
            PointCloudEncoder_3->i_frame_ = true;
        }

        PointCloudEncoder_3->frame_ID_++;

        std::cout << "Thread 3 Frame ID :" << PointCloudEncoder_3->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_3->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_3->point_count_data_vector_.clear ();
            PointCloudEncoder_3->point_count_data_vector_.reserve (octant_3->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_3->color_coder_.initializeEncoding ();
        PointCloudEncoder_3->color_coder_.setPointCount (static_cast<unsigned int> (octant_3->points.size ()));
        PointCloudEncoder_3->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_3->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_3->point_coder_.initializeEncoding ();
        PointCloudEncoder_3->point_coder_.setPointCount (static_cast<unsigned int> (octant_3->points.size ()));


        if( PointCloudEncoder_3->i_frame_){
            PointCloudEncoder_3->serializeTree( PointCloudEncoder_3->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_3->serializeTree( PointCloudEncoder_3->binary_tree_data_vector_,true);
        }



    }
}


void Alfa_Pc_Compress::compress_octant_4()
{
    std::cout << "Octant 4 Size: " << octant_4->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_4->getTreeDepth ());


    PointCloudEncoder_4->setInputCloud(octant_4);

    mux_oct_4.lock();
    PointCloudEncoder_4->addPointsFromInputCloud();
    mux_oct_4.unlock();

    if( PointCloudEncoder_4->getLeafCount()>0){
        PointCloudEncoder_4->cloud_with_color_ = false;
        PointCloudEncoder_4->cloud_with_color_ &= PointCloudEncoder_4->do_color_encoding_;

        PointCloudEncoder_4->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_4->getTreeDepth ());

        if ( PointCloudEncoder_4->i_frame_counter_++== PointCloudEncoder_4->i_frame_rate_)
        {
            PointCloudEncoder_4->i_frame_counter_ =0;
            PointCloudEncoder_4->i_frame_ = true;
        }

        PointCloudEncoder_4->frame_ID_++;

        std::cout << "Thread 4 Frame ID :" << PointCloudEncoder_4->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_4->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_4->point_count_data_vector_.clear ();
            PointCloudEncoder_4->point_count_data_vector_.reserve (octant_4->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_4->color_coder_.initializeEncoding ();
        PointCloudEncoder_4->color_coder_.setPointCount (static_cast<unsigned int> (octant_4->points.size ()));
        PointCloudEncoder_4->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_4->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_4->point_coder_.initializeEncoding ();
        PointCloudEncoder_4->point_coder_.setPointCount (static_cast<unsigned int> (octant_4->points.size ()));


        if( PointCloudEncoder_4->i_frame_){
            PointCloudEncoder_4->serializeTree( PointCloudEncoder_4->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_4->serializeTree( PointCloudEncoder_4->binary_tree_data_vector_,true);
        }



    }
}

void Alfa_Pc_Compress::compress_octant_5()
{
    std::cout << "Octant 5 Size: " << octant_5->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_5->getTreeDepth ());


    PointCloudEncoder_5->setInputCloud(octant_5);

    mux_oct_5.lock();
    PointCloudEncoder_5->addPointsFromInputCloud();
    mux_oct_5.unlock();

    if( PointCloudEncoder_5->getLeafCount()>0){
        PointCloudEncoder_5->cloud_with_color_ = false;
        PointCloudEncoder_5->cloud_with_color_ &= PointCloudEncoder_5->do_color_encoding_;

        PointCloudEncoder_5->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_5->getTreeDepth ());

        if ( PointCloudEncoder_5->i_frame_counter_++== PointCloudEncoder_5->i_frame_rate_)
        {
            PointCloudEncoder_5->i_frame_counter_ =0;
            PointCloudEncoder_5->i_frame_ = true;
        }

        PointCloudEncoder_5->frame_ID_++;

        std::cout << "Thread 5 Frame ID :" << PointCloudEncoder_5->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_5->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_5->point_count_data_vector_.clear ();
            PointCloudEncoder_5->point_count_data_vector_.reserve (octant_5->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_5->color_coder_.initializeEncoding ();
        PointCloudEncoder_5->color_coder_.setPointCount (static_cast<unsigned int> (octant_5->points.size ()));
        PointCloudEncoder_5->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_5->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_5->point_coder_.initializeEncoding ();
        PointCloudEncoder_5->point_coder_.setPointCount (static_cast<unsigned int> (octant_5->points.size ()));


        if( PointCloudEncoder_5->i_frame_){
            PointCloudEncoder_5->serializeTree( PointCloudEncoder_5->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_5->serializeTree( PointCloudEncoder_5->binary_tree_data_vector_,true);
        }



    }
}


void Alfa_Pc_Compress::compress_octant_6()
{
    std::cout << "Octant 6 Size: " << octant_6->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_6->getTreeDepth ());


    PointCloudEncoder_6->setInputCloud(octant_6);

    mux_oct_6.lock();
    PointCloudEncoder_6->addPointsFromInputCloud();
    mux_oct_6.unlock();

    if( PointCloudEncoder_6->getLeafCount()>0){
        PointCloudEncoder_6->cloud_with_color_ = false;
        PointCloudEncoder_6->cloud_with_color_ &= PointCloudEncoder_6->do_color_encoding_;

        PointCloudEncoder_6->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_6->getTreeDepth ());

        if ( PointCloudEncoder_6->i_frame_counter_++== PointCloudEncoder_6->i_frame_rate_)
        {
            PointCloudEncoder_6->i_frame_counter_ =0;
            PointCloudEncoder_6->i_frame_ = true;
        }

        PointCloudEncoder_6->frame_ID_++;

        std::cout << "Thread 6 Frame ID :" << PointCloudEncoder_6->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_6->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_6->point_count_data_vector_.clear ();
            PointCloudEncoder_6->point_count_data_vector_.reserve (octant_6->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_6->color_coder_.initializeEncoding ();
        PointCloudEncoder_6->color_coder_.setPointCount (static_cast<unsigned int> (octant_6->points.size ()));
        PointCloudEncoder_6->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_6->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_6->point_coder_.initializeEncoding ();
        PointCloudEncoder_6->point_coder_.setPointCount (static_cast<unsigned int> (octant_6->points.size ()));


        if( PointCloudEncoder_6->i_frame_){
            PointCloudEncoder_6->serializeTree( PointCloudEncoder_6->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_6->serializeTree( PointCloudEncoder_6->binary_tree_data_vector_,true);
        }



    }
}


void Alfa_Pc_Compress::compress_octant_7()
{
    std::cout << "Octant 7 Size: " << octant_7->size()<< std::endl;
    unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder_7->getTreeDepth ());


    PointCloudEncoder_7->setInputCloud(octant_7);

    mux_oct_7.lock();
    PointCloudEncoder_7->addPointsFromInputCloud();
    mux_oct_7.unlock();

    if( PointCloudEncoder_7->getLeafCount()>0){
        PointCloudEncoder_7->cloud_with_color_ = false;
        PointCloudEncoder_7->cloud_with_color_ &= PointCloudEncoder_7->do_color_encoding_;

        PointCloudEncoder_7->i_frame_ |= (recent_tree_depth !=   PointCloudEncoder_7->getTreeDepth ());

        if ( PointCloudEncoder_7->i_frame_counter_++== PointCloudEncoder_7->i_frame_rate_)
        {
            PointCloudEncoder_7->i_frame_counter_ =0;
            PointCloudEncoder_7->i_frame_ = true;
        }

        PointCloudEncoder_7->frame_ID_++;

        std::cout << "Thread 7 Frame ID :" << PointCloudEncoder_7->frame_ID_ <<std::endl;

        if (! PointCloudEncoder_7->do_voxel_grid_enDecoding_)
        {
            PointCloudEncoder_7->point_count_data_vector_.clear ();
            PointCloudEncoder_7->point_count_data_vector_.reserve (octant_7->points.size ());
        }

        // initialize color encoding
        PointCloudEncoder_7->color_coder_.initializeEncoding ();
        PointCloudEncoder_7->color_coder_.setPointCount (static_cast<unsigned int> (octant_7->points.size ()));
        PointCloudEncoder_7->color_coder_.setVoxelCount (static_cast<unsigned int> ( PointCloudEncoder_7->getLeafCount()));

        // initialize point encoding
        PointCloudEncoder_7->point_coder_.initializeEncoding ();
        PointCloudEncoder_7->point_coder_.setPointCount (static_cast<unsigned int> (octant_7->points.size ()));


        if( PointCloudEncoder_7->i_frame_){
            PointCloudEncoder_7->serializeTree( PointCloudEncoder_7->binary_tree_data_vector_,false);
        }else{
            PointCloudEncoder_7->serializeTree( PointCloudEncoder_7->binary_tree_data_vector_,true);
        }

         std::cout << "Acebei Serialize 7 \n";


    }
}






void Alfa_Pc_Compress::run_worker_thread(int thread_number,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud){
    std::cout << "//////////////// encodePointCloud_2 TESTE THREADING //////////////////\n";

    for(int i = (input_cloud->size()/number_threads)*thread_number;i<= (input_cloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZRGB point = (*input_cloud)[i];

        //push point to cluster
        if(thread_number == 0)
        {
            mutex_cluster.lock();
            cluster0->push_back(point);
            mutex_cluster.unlock();


        }else if (thread_number == 1)
        {
            mutex_cluster.lock();
            cluster1->push_back(point);
            mutex_cluster.unlock();
        }

    }



    if(thread_number == 0){
        std::cout << "Cluster 0 Size: " << cluster1->size()<< std::endl;
        unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder->getTreeDepth ());

        PointCloudEncoder->setInputCloud(cluster0);
        PointCloudEncoder->addPointsFromInputCloud();

        if(PointCloudEncoder->getLeafCount()>0){
            PointCloudEncoder->cloud_with_color_ = false;
            PointCloudEncoder->cloud_with_color_ &=PointCloudEncoder->do_color_encoding_;

            PointCloudEncoder->i_frame_ |= (recent_tree_depth !=  PointCloudEncoder->getTreeDepth ());

            if (PointCloudEncoder->i_frame_counter_++==PointCloudEncoder->i_frame_rate_)
            {
                PointCloudEncoder->i_frame_counter_ =0;
                PointCloudEncoder->i_frame_ = true;
            }

            PointCloudEncoder->frame_ID_++;

            std::cout << "Thread 0 Frame ID :\n" <<PointCloudEncoder->frame_ID_ <<std::endl;

            if (!PointCloudEncoder->do_voxel_grid_enDecoding_)
            {
                PointCloudEncoder->point_count_data_vector_.clear ();
                PointCloudEncoder->point_count_data_vector_.reserve (cluster0->points.size ());
            }

            // initialize color encoding
            PointCloudEncoder->color_coder_.initializeEncoding ();
            PointCloudEncoder->color_coder_.setPointCount (static_cast<unsigned int> (cluster0->points.size ()));
            PointCloudEncoder->color_coder_.setVoxelCount (static_cast<unsigned int> (PointCloudEncoder->getLeafCount()));

            // initialize point encoding
            PointCloudEncoder->point_coder_.initializeEncoding ();
            PointCloudEncoder->point_coder_.setPointCount (static_cast<unsigned int> (cluster0->points.size ()));


            if(PointCloudEncoder->i_frame_){
                PointCloudEncoder->serializeTree(PointCloudEncoder->binary_tree_data_vector_,false);
            }else{
                PointCloudEncoder->serializeTree(PointCloudEncoder->binary_tree_data_vector_,true);
            }



        }
    }else if (thread_number == 1) {
             std::cout << "Thread 1\n";
             std::cout << "Cluster 1 Size: " << cluster1->size()<< std::endl;

             unsigned char recent_tree_depth = static_cast<unsigned char> (PointCloudEncoder1->getTreeDepth ());

             PointCloudEncoder1->setInputCloud(cluster1);
             PointCloudEncoder1->addPointsFromInputCloud();

             if(PointCloudEncoder1->getLeafCount()>0){
                 PointCloudEncoder1->cloud_with_color_ = false;
                 PointCloudEncoder1->cloud_with_color_ &=PointCloudEncoder1->do_color_encoding_;

                 PointCloudEncoder1->i_frame_ |= (recent_tree_depth !=  PointCloudEncoder1->getTreeDepth ());

                 if (PointCloudEncoder1->i_frame_counter_++==PointCloudEncoder1->i_frame_rate_)
                 {
                     PointCloudEncoder1->i_frame_counter_ =0;
                     PointCloudEncoder1->i_frame_ = true;
                 }

                 PointCloudEncoder1->frame_ID_++;

                 std::cout << "Thread 1 Frame ID :\n" <<PointCloudEncoder1->frame_ID_ <<std::endl;

                 if (!PointCloudEncoder1->do_voxel_grid_enDecoding_)
                 {
                     PointCloudEncoder1->point_count_data_vector_.clear ();
                     PointCloudEncoder1->point_count_data_vector_.reserve (cluster1->points.size ());
                 }

                 // initialize color encoding
                 PointCloudEncoder1->color_coder_.initializeEncoding ();
                 PointCloudEncoder1->color_coder_.setPointCount (static_cast<unsigned int> (cluster1->points.size ()));
                 PointCloudEncoder1->color_coder_.setVoxelCount (static_cast<unsigned int> (PointCloudEncoder1->getLeafCount()));

                 // initialize point encoding
                 PointCloudEncoder1->point_coder_.initializeEncoding ();
                 PointCloudEncoder1->point_coder_.setPointCount (static_cast<unsigned int> (cluster1->points.size ()));


                 if(PointCloudEncoder1->i_frame_){
                     PointCloudEncoder1->serializeTree(PointCloudEncoder1->binary_tree_data_vector_,false);
                 }else{
                     PointCloudEncoder1->serializeTree(PointCloudEncoder1->binary_tree_data_vector_,true);
                 }

             }


    }



    /// continuar.....

}

void Alfa_Pc_Compress::run_worker_thread_odd_even(int thread_number,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{

    if(thread_number == 0){ // impar
        if(input_cloud->header.seq % 2){
            std::cout << "Thread 0\n";
            std::cout << "SEQ: \n" << input_cloud->header.seq << std::endl;
            std::stringstream compressed_data_0;
            PointCloudEncoder->encodePointCloud_2(in_cloud,compressed_data_0);


            output_compressed.data = compressed_data_0.str();

            mutex_compressed_data1.lock();
            publish_pointcloud(output_compressed);
            mutex_compressed_data1.unlock();

        }
    }
    if(thread_number == 1){ // par
        if(!(input_cloud->header.seq % 2)){
            std::cout << "Thread 1\n";
            std::cout << "SEQ: \n" << input_cloud->header.seq % 2 << std::endl;
            std::stringstream compressed_data_1;
            PointCloudEncoder1->encodePointCloud_2(in_cloud,compressed_data_1);


            output_compressed1.data = compressed_data_1.str();
            mutex_compressed_data2.lock();
            publish_pointcloud(output_compressed1);
            mutex_compressed_data2.unlock();

        }
    }

    // nao funciona ???????? Decoder ?????



}



void Alfa_Pc_Compress::divide_in_octants_2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) // PCL, mais lenta
{

    //demora mais
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*input_cloud,minPt,maxPt);

    float mid_x,mid_y,mid_z;

    mid_x = (maxPt.x + minPt.x)/2 ;
    mid_y = (maxPt.y + minPt.y)/2 ;
    mid_z = (maxPt.z + minPt.z)/2 ;

    printf("%f \n",maxPt.x);
    printf("%f \n",maxPt.y);
    printf("%f \n",maxPt.z);
    printf("%f \n",minPt.x);
    printf("%f \n",minPt.y);
    printf("%f \n",minPt.z);

    printf("%f \n",mid_x);
    printf("%f \n",mid_y);
    printf("%f \n",mid_z);



    for (int i=0;i<input_cloud->size();i++) {
        pcl::PointXYZRGB point = (*input_cloud)[i];

        if(point.x <= mid_x){
            if(point.y <= mid_y){
                if(point.z <= mid_z)
                    octant_0->push_back(point);
                else
                    octant_1->push_back(point);
            }
            else {
                if(point.z <= mid_z)
                    octant_2->push_back(point);
                else
                    octant_3->push_back(point);
            }
        }
        else {
            if(point.y <= mid_y){
                if(point.z <= mid_z)
                    octant_4->push_back(point);
                else
                    octant_5->push_back(point);
            }
            else{
                if(point.z <= mid_z)
                    octant_6->push_back(point);
                else
                    octant_7->push_back(point);
            }
        }


    }


}


//for(int i=0; i<PointCloudEncoder_1->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_1->binary_tree_data_vector_[i]);

//printf("\n------------------------------------------------------------\n");
//for(int i=0; i<PointCloudEncoder_2->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_2->binary_tree_data_vector_[i]);

//printf("\n------------------------------------------------------------\n");
//for(int i=0; i<PointCloudEncoder_3->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_3->binary_tree_data_vector_[i]);

//printf("\n------------------------------------------------------------\n");
//for(int i=0; i<PointCloudEncoder_4->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_4->binary_tree_data_vector_[i]);

//printf("\n------------------------------------------------------------\n");
//for(int i=0; i<PointCloudEncoder_5->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_5->binary_tree_data_vector_[i]);

//printf("\n------------------------------------------------------------\n");
//for(int i=0; i<PointCloudEncoder_6->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_6->binary_tree_data_vector_[i]);

//printf("\n------------------------------------------------------------\n");
//for(int i=0; i<PointCloudEncoder_7->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_7->binary_tree_data_vector_[i]);

//printf("\n------------------------------------------------------------\n");
//for(int i=0; i<PointCloudEncoder_0->binary_tree_data_vector_.size();i++)
//    printf("%i --> %x \n",i,PointCloudEncoder_0->binary_tree_data_vector_[i]);
