#include "my_octree.h"
#define MAX_DEPTH 16


Octree::Octree()
{
    root = new OctreeNode;
    root->leafs = 0;
    root->is_leaf = false;



    for(int i=0;i<8;i++)
        root->children[i] = 0;


    point_count = 0;
    leaf_count = 0;
    branch_count = 0;

    memset(child,0,sizeof(child));


    printf("Sizeof child = %d\n", sizeof(child));

    resolution = 0.03;
    depth = 0;


}

Octree::Octree(OctreeNode node, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{

}

Octree::~Octree(){
    delete root;

}

void Octree::calculate_bounds(OctreeNode node)
{
    BoundingBox new_bb;

//    new_bb.max_x = node.points_list[0]._PointXYZRGB::x;
//    new_bb.max_y = node.points_list[0]._PointXYZRGB::y;
//    new_bb.max_z = node.points_list[0]._PointXYZRGB::z;
//    new_bb.min_x = node.points_list[0]._PointXYZRGB::x;
//    new_bb.min_y = node.points_list[0]._PointXYZRGB::y;
//    new_bb.min_z = node.points_list[0]._PointXYZRGB::z;

//    for(unsigned int i = 0;i<node.points_list.size();i++){
//        if(node.points_list[i]._PointXYZRGB::x < new_bb.min_x)
//            new_bb.min_x = node.points_list[i]._PointXYZRGB::x;
//        if(node.points_list[i]._PointXYZRGB::y < new_bb.min_y)
//            new_bb.min_y = node.points_list[i]._PointXYZRGB::y;
//        if(node.points_list[i]._PointXYZRGB::z < new_bb.min_z)
//            new_bb.min_z = node.points_list[i]._PointXYZRGB::x;
//        if(node.points_list[i]._PointXYZRGB::x > new_bb.max_x)
//            new_bb.max_x = node.points_list[i]._PointXYZRGB::x;
//        if(node.points_list[i]._PointXYZRGB::y > new_bb.max_y)
//            new_bb.max_y = node.points_list[i]._PointXYZRGB::y;
//        if(node.points_list[i]._PointXYZRGB::z > new_bb.max_z)
//            new_bb.max_z = node.points_list[i]._PointXYZRGB::z;
//    }



}

bool Octree::build_octree(std::vector<pcl::PointXYZRGB> points,BoundingBox bounds)
{
    //declarations
    printf("Point Count = %d\n", points.size());

    //check if it is leaf | If it reach max. depth

    if(depth >= MAX_DEPTH){
        //store points in node
        point_count = points.size();



        //************************
        OctreeNode *new_leaf;
        new_leaf->points_list = points;
        new_leaf->is_leaf = true;
        for(int i=0;i<8;i++)
            new_leaf->children[i] = 0;
        //***********************


        leaf_count++;

        //terminate recursion
        return true;
    }


    // Classify each point to a child node
    // check center of node or bb of node
    pcl::PointXYZRGB center;
    center._PointXYZRGB::x = (bounds.max_x - bounds.min_x) * 0.5;
    center._PointXYZRGB::y = (bounds.max_y - bounds.min_y) * 0.5;
    center._PointXYZRGB::z = (bounds.max_z - bounds.min_z) * 0.5;

    unsigned int child_point_counts[8];

    for(uint8_t i=0;i<points.size();i++){


        pcl::PointXYZRGB point = points[i];

        // check which child node each point belongs to
        point._PointXYZRGB::rgba = 0;// test, just to save code

        if(point._PointXYZRGB::x > center._PointXYZRGB::x)
            point._PointXYZRGB::rgba |= 1;
        if(point._PointXYZRGB::y > center._PointXYZRGB::y)
            point._PointXYZRGB::rgba |= 2;
        if(point._PointXYZRGB::z > center._PointXYZRGB::z)
            point._PointXYZRGB::rgba |= 4;


        child_point_counts[point._PointXYZRGB::rgba]++;

    }

    // build recursive

    for (int i=0;i<8;i++) {

        //if there aren't any points for this child
        if(!child_point_counts[i])
            continue;

        // allocate the new child
        child[i] = new Octree;

        //allocate a list of points that were coded for this child
        pcl::PointXYZRGB **new_point_list = new pcl::PointXYZRGB *[child_point_counts[i]];


        pcl::PointXYZRGB **ptr = new_point_list;
        std::vector<pcl::PointXYZRGB> new_points;

        for (unsigned int j=0;j<points.size();j++)
        {
            if (points[j]._PointXYZRGB::rgba == i)
            {
                *ptr = &points[j];
                new_points.push_back(points[j]);
                ptr++;
            }
        }

        int new_count=0;

        for(int j = 0;i<child_point_counts[i];j++){
            //remove duplicates new_points;
        }

        //calculate bounding box
        BoundingBox new_box{0,0,0,0,0,0};


        for (int i=0;i<new_points.size();i++) {
            if(new_points[i]._PointXYZRGB::x < new_box.min_x)
                new_box.min_x = new_points[i]._PointXYZRGB::x;

            if(new_points[i]._PointXYZRGB::y < new_box.min_y)
                new_box.min_y = new_points[i]._PointXYZRGB::y;

            if(new_points[i]._PointXYZRGB::z < new_box.min_z)
                new_box.min_z = new_points[i]._PointXYZRGB::z;

            if(new_points[i]._PointXYZRGB::x > new_box.max_x)
                new_box.max_x = new_points[i]._PointXYZRGB::x;

            if(new_points[i]._PointXYZRGB::y > new_box.max_y)
                new_box.max_y = new_points[i]._PointXYZRGB::y;

            if(new_points[i]._PointXYZRGB::z > new_box.max_z)
                new_box.max_z = new_points[i]._PointXYZRGB::z;
        }


        //recursive
        child[i]->build_octree(new_points,new_box);

        delete[] new_point_list;
        new_points.clear();




        //new depth
        depth++;
        printf("depth = %d", depth);



    }
    return true;





}


