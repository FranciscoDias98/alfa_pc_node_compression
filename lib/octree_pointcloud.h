/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */

#ifndef PCL_OCTREE_POINTCLOUD_H
#define PCL_OCTREE_POINTCLOUD_H

#include <pcl/octree/octree_base.h>
//#include "octree2buf_base.h"
#include <pcl/octree/octree_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud class
     *  \note Octree implementation for pointclouds. Only indices are stored by the octree leaf nodes (zero-copy).
     *  \note The octree pointcloud class needs to be initialized with its voxel resolution. Its bounding box is automatically adjusted
     *  \note according to the pointcloud dimension or it can be predefined.
     *  \note Note: The tree depth equates to the resolution and the bounding box dimensions of the octree.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \note typename: LeafContainerT:  leaf node container (
     *  \note typename: BranchContainerT:  branch node container
     *  \note typename: OctreeT: octree implementation ()
     *  \ingroup octree
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafContainerT = OctreeContainerPointIndices,
        typename BranchContainerT = OctreeContainerEmpty,
        typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> >

    class OctreePointCloud : public OctreeT
    {
        // iterators are friends
        friend class OctreeIteratorBase<OctreeT> ;
        friend class OctreeDepthFirstIterator<OctreeT> ;
        friend class OctreeBreadthFirstIterator<OctreeT> ;
        friend class OctreeLeafNodeIterator<OctreeT> ;

      public:
        typedef OctreeT Base;

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        // Octree default iterators
        typedef OctreeDepthFirstIterator<OctreeT> Iterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstIterator;

        // Octree leaf node iterators
        typedef OctreeLeafNodeIterator<OctreeT> LeafNodeIterator;
        typedef const OctreeLeafNodeIterator<OctreeT> ConstLeafNodeIterator;

        // Octree depth-first iterators
        typedef OctreeDepthFirstIterator<OctreeT> DepthFirstIterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstDepthFirstIterator;

        // Octree breadth-first iterators
        typedef OctreeBreadthFirstIterator<OctreeT> BreadthFirstIterator;
        typedef const OctreeBreadthFirstIterator<OctreeT> ConstBreadthFirstIterator;

        /** \brief Octree pointcloud constructor.
         * \param[in] resolution_arg octree resolution at lowest octree level
         */
        OctreePointCloud (const double resolution_arg);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreePointCloud ();

        // public typedefs
        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // public typedefs for single/double buffering
        typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeBase<LeafContainerT> > SingleBuffer;
       // typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT, Octree2BufBase<LeafContainerT> > DoubleBuffer;

        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> > ConstPtr;

        // Eigen aligned allocator
        typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > AlignedPointTVector;
        typedef std::vector<PointXYZ, Eigen::aligned_allocator<PointXYZ> > AlignedPointXYZVector;

        /** \brief Provide a pointer to the input data set.
         * \param[in] cloud_arg the const boost shared pointer to a PointCloud message
         * \param[in] indices_arg the point indices subset that is to be used from \a cloud - if 0 the whole point cloud is used
         */
        inline void setInputCloud (const PointCloudConstPtr &cloud_arg,
            const IndicesConstPtr &indices_arg = IndicesConstPtr ())
        {
          input_ = cloud_arg;
          indices_ = indices_arg;
        }

        /** \brief Get a pointer to the vector of indices used.
         * \return pointer to vector of indices used.
         */
        inline IndicesConstPtr const getIndices () const
        {
          return (indices_);
        }

        /** \brief Get a pointer to the input point cloud dataset.
         * \return pointer to pointcloud input class.
         */
        inline PointCloudConstPtr getInputCloud () const
        {
          printf("Ola octree_pointcloud 2\n");
          return (input_);
        }

        /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
         * \param[in] eps precision (error bound) for nearest neighbors searches
         */
        inline void setEpsilon (double eps)
        {
          epsilon_ = eps;
        }

        /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
        inline double getEpsilon () const
        {
          return (epsilon_);
        }

        /** \brief Set/change the octree voxel resolution
         * \param[in] resolution_arg side length of voxels at lowest tree level
         */
        inline void setResolution (double resolution_arg)
        {
          // octree needs to be empty to change its resolution
          assert( this->leaf_count_ == 0);

          resolution_ = resolution_arg;

          getKeyBitSize ();
        }

        /** \brief Get octree voxel resolution
         * \return voxel resolution at lowest tree level
         */
        inline double getResolution () const
        {
          return (resolution_);
        }

        /** \brief Get the maximum depth of the octree.
         *  \return depth_arg: maximum depth of octree
         * */
        inline unsigned int getTreeDepth () const
        {
          return this->octree_depth_;
        }

        /** \brief Add points from input point cloud to octree. */
        void
        addPointsFromInputCloud ();



        void addPointsFromInputCloud_2(){
            size_t i;
              printf("addPointsFromInputCloud_2\n");
              if (this->indices_)
              {
                for (std::vector<int>::const_iterator current = this->indices_->begin (); current != this->indices_->end (); ++current)
                {
                  assert( (*current>=0) && (*current < static_cast<int> (this->input_->points.size ())));

                  if (isFinite (this->input_->points[*current]))
                  {
                    // add points to octree
                    this->addPointIdx_2 (*current);
                  }
                }
              }
              else
              {
                for (i = 0; i < this->input_->points.size (); i++)
                {
                  if (isFinite (this->input_->points[i]))
                  {
                    // add points to octree
                    this->addPointIdx_2 (static_cast<unsigned int> (i));
                  }
                }
              }
              printf("%f \n",min_x_);
              printf("%f \n",min_y_);
              printf("%f \n",min_z_);
              printf("%f \n",max_x_);
              printf("%f \n",max_y_);
              printf("%f \n",max_z_);
        }

        /** \brief Add point at given index from input point cloud to octree. Index will be also added to indices vector.
         * \param[in] point_idx_arg index of point to be added
         * \param[in] indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         */
        void
        addPointFromCloud (const int point_idx_arg, IndicesPtr indices_arg);

        /** \brief Add point simultaneously to octree and input point cloud.
         *  \param[in] point_arg point to be added
         *  \param[in] cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg);

        /** \brief Add point simultaneously to octree and input point cloud. A corresponding index will be added to the indices vector.
         * \param[in] point_arg point to be added
         * \param[in] cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         * \param[in] indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg, IndicesPtr indices_arg);

        /** \brief Check if voxel at given point exist.
         * \param[in] point_arg point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const PointT& point_arg) const;

        /** \brief Delete the octree structure and its leaf nodes.
         * */
        void deleteTree ()
        {
          // reset bounding box
          min_x_ = min_y_ = max_y_ = min_z_ = max_z_ = 0;
          this->bounding_box_defined_ = false;

          OctreeT::deleteTree ();
        }

        /** \brief Check if voxel at given point coordinates exist.
         * \param[in] point_x_arg X coordinate of point to be checked
         * \param[in] point_y_arg Y coordinate of point to be checked
         * \param[in] point_z_arg Z coordinate of point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const double point_x_arg, const double point_y_arg, const double point_z_arg) const;

        /** \brief Check if voxel at given point from input cloud exist.
         * \param[in] point_idx_arg point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const int& point_idx_arg) const;

        /** \brief Get a PointT vector of centers of all occupied voxels.
         * \param[out] voxel_center_list_arg results are written to this vector of PointT elements
         * \return number of occupied voxels
         */
        int
        getOccupiedVoxelCenters (AlignedPointTVector &voxel_center_list_arg) const;

        /** \brief Get a PointT vector of centers of voxels intersected by a line segment.
         * This returns a approximation of the actual intersected voxels by walking
         * along the line with small steps. Voxels are ordered, from closest to
         * furthest w.r.t. the origin.
         * \param[in] origin origin of the line segment
         * \param[in] end end of the line segment
         * \param[out] voxel_center_list results are written to this vector of PointT elements
         * \param[in] precision determines the size of the steps: step_size = octree_resolution x precision
         * \return number of intersected voxels
         */
        int
        getApproxIntersectedVoxelCentersBySegment (
            const Eigen::Vector3f& origin, const Eigen::Vector3f& end,
            AlignedPointTVector &voxel_center_list, float precision = 0.2);

        /** \brief Delete leaf node / voxel at given point
         * \param[in] point_arg point addressing the voxel to be deleted.
         */
        void
        deleteVoxelAtPoint (const PointT& point_arg);

        /** \brief Delete leaf node / voxel at given point from input cloud
         *  \param[in] point_idx_arg index of point addressing the voxel to be deleted.
         */
        void
        deleteVoxelAtPoint (const int& point_idx_arg);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Bounding box methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Investigate dimensions of pointcloud data set and define corresponding bounding box for octree. */
        void
        defineBoundingBox ();

        /** \brief Define bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] min_x_arg X coordinate of lower bounding box corner
         * \param[in] min_y_arg Y coordinate of lower bounding box corner
         * \param[in] min_z_arg Z coordinate of lower bounding box corner
         * \param[in] max_x_arg X coordinate of upper bounding box corner
         * \param[in] max_y_arg Y coordinate of upper bounding box corner
         * \param[in] max_z_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double min_x_arg, const double min_y_arg, const double min_z_arg,
                           const double max_x_arg, const double max_y_arg, const double max_z_arg);

        /** \brief Define bounding box for octree
         * \note Lower bounding box point is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] max_x_arg X coordinate of upper bounding box corner
         * \param[in] max_y_arg Y coordinate of upper bounding box corner
         * \param[in] max_z_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double max_x_arg, const double max_y_arg, const double max_z_arg);

        /** \brief Define bounding box cube for octree
         * \note Lower bounding box corner is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] cubeLen_arg side length of bounding box cube.
         */
        void
        defineBoundingBox (const double cubeLen_arg);

        /** \brief Get bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] min_x_arg X coordinate of lower bounding box corner
         * \param[in] min_y_arg Y coordinate of lower bounding box corner
         * \param[in] min_z_arg Z coordinate of lower bounding box corner
         * \param[in] max_x_arg X coordinate of upper bounding box corner
         * \param[in] max_y_arg Y coordinate of upper bounding box corner
         * \param[in] max_z_arg Z coordinate of upper bounding box corner
         */
        void
        getBoundingBox (double& min_x_arg, double& min_y_arg, double& min_z_arg,
                        double& max_x_arg, double& max_y_arg, double& max_z_arg) const;

        /** \brief Calculates the squared diameter of a voxel at given tree depth
         * \param[in] tree_depth_arg depth/level in octree
         * \return squared diameter
         */
        double
        getVoxelSquaredDiameter (unsigned int tree_depth_arg) const;

        /** \brief Calculates the squared diameter of a voxel at leaf depth
         * \return squared diameter
         */
        inline double
        getVoxelSquaredDiameter () const
        {
          return getVoxelSquaredDiameter (this->octree_depth_);
        }

        /** \brief Calculates the squared voxel cube side length at given tree depth
         * \param[in] tree_depth_arg depth/level in octree
         * \return squared voxel cube side length
         */
        double
        getVoxelSquaredSideLen (unsigned int tree_depth_arg) const;

        /** \brief Calculates the squared voxel cube side length at leaf level
         * \return squared voxel cube side length
         */
        inline double getVoxelSquaredSideLen () const
        {
          return getVoxelSquaredSideLen (this->octree_depth_);
        }

        /** \brief Generate bounds of the current voxel of an octree iterator
         * \param[in] iterator: octree iterator
         * \param[out] min_pt lower bound of voxel
         * \param[out] max_pt upper bound of voxel
         */
        inline void
        getVoxelBounds (const OctreeIteratorBase<OctreeT>& iterator, Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt) const
        {
          this->genVoxelBoundsFromOctreeKey (iterator.getCurrentOctreeKey (),
              iterator.getCurrentOctreeDepth (), min_pt, max_pt);
        }

        /** \brief Enable dynamic octree structure
         *  \note Leaf nodes are kept as close to the root as possible and are only expanded if the number of DataT objects within a leaf node exceeds a fixed limit.
         *  \param maxObjsPerLeaf: maximum number of DataT objects per leaf
         * */
        inline void
        enableDynamicDepth ( size_t maxObjsPerLeaf )
        {
          assert(this->leaf_count_==0);
          max_objs_per_leaf_ = maxObjsPerLeaf;

          this->dynamic_depth_enabled_ = static_cast<bool> (max_objs_per_leaf_>0);
        }


      protected:

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] point_idx_arg the index representing the point in the dataset given by \a setInputCloud to be added
         */
        virtual void
        addPointIdx (const int point_idx_arg);

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] leaf_node to be expanded
         * \param[in] parent_branch parent of leaf node to be expanded
         * \param[in] child_idx child index of leaf node (in parent branch)
         * \param[in] depth_mask of leaf node to be expanded
         */
        void
        expandLeafNode (LeafNode* leaf_node, BranchNode* parent_branch, unsigned char child_idx, unsigned int depth_mask);

        /** \brief Get point at index from input pointcloud dataset
         * \param[in] index_arg index representing the point in the dataset given by \a setInputCloud
         * \return PointT from input pointcloud dataset
         */
        const PointT&
        getPointByIndex (const unsigned int index_arg) const;

        /** \brief Find octree leaf node at a given point
         * \param[in] point_arg query point
         * \return pointer to leaf node. If leaf node does not exist, pointer is 0.
         */
        LeafContainerT*
        findLeafAtPoint (const PointT& point_arg) const
        {
          OctreeKey key;

          // generate key for point
          this->genOctreeKeyforPoint (point_arg, key);

          return (this->findLeaf (key));
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Define octree key setting and octree depth based on defined bounding box. */
        void
        getKeyBitSize ();

        /** \brief Grow the bounding box/octree until point fits
         * \param[in] point_idx_arg point that should be within bounding box;
         */


        //test %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        void
        adoptBoundingBoxToPoint (const PointT& point_idx_arg);

        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        /** \brief Checks if given point is within the bounding box of the octree
         * \param[in] point_idx_arg point to be checked for bounding box violations
         * \return "true" - no bound violation
         */
        inline bool isPointWithinBoundingBox (const PointT& point_idx_arg) const
        {
          return (! ( (point_idx_arg.x < min_x_) || (point_idx_arg.y < min_y_)
                   || (point_idx_arg.z < min_z_) || (point_idx_arg.x >= max_x_)
                   || (point_idx_arg.y >= max_y_) || (point_idx_arg.z >= max_z_)));
        }

        /** \brief Generate octree key for voxel at a given point
         * \param[in] point_arg the point addressing a voxel
         * \param[out] key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const PointT & point_arg,
            OctreeKey &key_arg) const;

        /** \brief Generate octree key for voxel at a given point
         * \param[in] point_x_arg X coordinate of point addressing a voxel
         * \param[in] point_y_arg Y coordinate of point addressing a voxel
         * \param[in] point_z_arg Z coordinate of point addressing a voxel
         * \param[out] key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const double point_x_arg, const double point_y_arg, const double point_z_arg,
                              OctreeKey & key_arg) const;

        /** \brief Virtual method for generating octree key for a given point index.
         * \note This method enables to assign indices to leaf nodes during octree deserialization.
         * \param[in] data_arg index value representing a point in the dataset given by \a setInputCloud
         * \param[out] key_arg write octree key to this reference
         * \return "true" - octree keys are assignable
         */
        virtual bool
        genOctreeKeyForDataT (const int& data_arg, OctreeKey & key_arg) const;

        /** \brief Generate a point at center of leaf node voxel
         * \param[in] key_arg octree key addressing a leaf node.
         * \param[out] point_arg write leaf node voxel center to this point reference
         */
        void
        genLeafNodeCenterFromOctreeKey (const OctreeKey & key_arg,
            PointT& point_arg) const;

        /** \brief Generate a point at center of octree voxel at given tree level
         * \param[in] key_arg octree key addressing an octree node.
         * \param[in] tree_depth_arg octree depth of query voxel
         * \param[out] point_arg write leaf node center point to this reference
         */
        void
        genVoxelCenterFromOctreeKey (const OctreeKey & key_arg,
            unsigned int tree_depth_arg, PointT& point_arg) const;

        /** \brief Generate bounds of an octree voxel using octree key and tree depth arguments
         * \param[in] key_arg octree key addressing an octree node.
         * \param[in] tree_depth_arg octree depth of query voxel
         * \param[out] min_pt lower bound of voxel
         * \param[out] max_pt upper bound of voxel
         */
        void
        genVoxelBoundsFromOctreeKey (const OctreeKey & key_arg,
            unsigned int tree_depth_arg, Eigen::Vector3f &min_pt,
            Eigen::Vector3f &max_pt) const;

        /** \brief Recursively search the tree for all leaf nodes and return a vector of voxel centers.
         * \param[in] node_arg current octree node to be explored
         * \param[in] key_arg octree key addressing a leaf node.
         * \param[out] voxel_center_list_arg results are written to this vector of PointT elements
         * \return number of voxels found
         */
        int
        getOccupiedVoxelCentersRecursive (const BranchNode* node_arg,
            const OctreeKey& key_arg,
            AlignedPointTVector &voxel_center_list_arg) const;







        void addPointIdx_2 (const int point_idx_arg){
            OctreeKey key;

              assert (point_idx_arg < static_cast<int> (this->input_->points.size ()));

              const PointT& point = this->input_->points[point_idx_arg];
              printf("*************************************************\n");
              printf("Point ID: %d | (%f, %f , %f)\n",point_idx_arg,point.x,point.y,point.z);
              // make sure bounding box is big enough
              this->adoptBoundingBoxToPoint_2 (point);


             printf(" ------------ BB ------------\n");
             printf("MIN: %f , %f , %f\n",this->min_x_,this->min_y_,this->min_z_);
             printf("MAX: %f , %f , %f\n",this->max_x_,this->max_y_,this->max_z_);
             printf(" ----------------------------\n");
             // generate key
             this->genOctreeKeyforPoint (point, key);

             printf("Key: %f, %f, %f\n",key.x,key.y,key.z);

              LeafNode* leaf_node;
              BranchNode* parent_branch_of_leaf_node;
              unsigned int depth_mask = this->createLeafRecursive_2(key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);

              // printf("Depth mask: %d\n", depth_mask);
              //printf("dynamic_depth_enabled_: %d\n", this->dynamic_depth_enabled_); // <--- 0 ;
              if (this->dynamic_depth_enabled_ && depth_mask)
              {
                // get amount of objects in leaf container
                size_t leaf_obj_count = (*leaf_node)->getSize ();

                while  (leaf_obj_count>=this->max_objs_per_leaf_ && depth_mask)
                {
                  // index to branch child

                  unsigned char child_idx = key.getChildIdxWithDepthMask (depth_mask*2);
                  //printf("Point: (%f , %f , %f ) | Child idx: %x \n",key.x,key.y,key.z,child_idx);
                  this->expandLeafNode (leaf_node,
                                  parent_branch_of_leaf_node,
                                  child_idx,
                                  depth_mask);

                  depth_mask = this->createLeafRecursive_2 (key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);
                  leaf_obj_count = (*leaf_node)->getSize ();
                }

              }

              (*leaf_node)->addPointIndex (point_idx_arg);
              printf("*************************************************\n");
        }

        void
        adoptBoundingBoxToPoint_2 (const PointT& point_idx_arg){

             const float minValue = std::numeric_limits<float>::epsilon ();
             // increase octree size until point fits into bounding box
             while (true)
             {
               bool bLowerBoundViolationX = (point_idx_arg.x < this->min_x_);
               bool bLowerBoundViolationY = (point_idx_arg.y < this->min_y_);
               bool bLowerBoundViolationZ = (point_idx_arg.z < this->min_z_);

               bool bUpperBoundViolationX = (point_idx_arg.x >= this->max_x_);
               bool bUpperBoundViolationY = (point_idx_arg.y >= this->max_y_);
               bool bUpperBoundViolationZ = (point_idx_arg.z >= this->max_z_);

               // do we violate any bounds?
               if (bLowerBoundViolationX || bLowerBoundViolationY || bLowerBoundViolationZ || bUpperBoundViolationX
                   || bUpperBoundViolationY || bUpperBoundViolationZ || (!(this->bounding_box_defined_)) )
               {
                 printf(" ------------ BB violation ------------\n");

                 if (this->bounding_box_defined_)
                 {

                   double octreeSideLen;
                   unsigned char child_idx;


                   // octree not empty - we add another tree level and thus increase its size by a factor of 2*2*2
                   child_idx = static_cast<unsigned char> (((!bUpperBoundViolationX) << 2) | ((!bUpperBoundViolationY) << 1)
                       | ((!bUpperBoundViolationZ)));


                   printf("Point: (%f , %f , %f ) | Child idx: %x \n",point_idx_arg.x,point_idx_arg.y,point_idx_arg.z,child_idx);

                   BranchNode* newRootBranch;

                   newRootBranch = new BranchNode();
                   this->branch_count_++;

                   this->setBranchChildPtr (*newRootBranch, child_idx, this->root_node_);

                   this->root_node_ = newRootBranch;

                   octreeSideLen = static_cast<double> (1 << this->octree_depth_) * this->resolution_;
                   printf("Octree side len: %f\n", octreeSideLen);

                   if (!bUpperBoundViolationX)
                     this->min_x_ -= octreeSideLen;

                   if (!bUpperBoundViolationY)
                     this->min_y_ -= octreeSideLen;

                   if (!bUpperBoundViolationZ)
                     this->min_z_ -= octreeSideLen;

                   // configure tree depth of octree
                   this->octree_depth_++;
                   this->setTreeDepth (this->octree_depth_);
                   printf("OCtree Depth: %d\n",this->octree_depth_);
                   // recalculate bounding box width
                   octreeSideLen = static_cast<double> (1 << this->octree_depth_) * this->resolution_ - minValue;
                   printf("Octree side len recalc: %f\n", octreeSideLen);
                   // increase octree bounding box
                   this->max_x_ = this->min_x_ + octreeSideLen;
                   this->max_y_ = this->min_y_ + octreeSideLen;
                   this->max_z_ = this->min_z_ + octreeSideLen;
                   printf("---------------------------------------\n");

                 }
                 // bounding box is not defined - set it to point position
                 else
                 {
                   // octree is empty - we set the center of the bounding box to our first pixel
                   this->min_x_ = point_idx_arg.x - this->resolution_ / 2;
                   this->min_y_ = point_idx_arg.y - this->resolution_ / 2;
                   this->min_z_ = point_idx_arg.z - this->resolution_ / 2;

                   this->max_x_ = point_idx_arg.x + this->resolution_ / 2;
                   this->max_y_ = point_idx_arg.y + this->resolution_ / 2;
                   this->max_z_ = point_idx_arg.z + this->resolution_ / 2;

                   this->getKeyBitSize ();

                   this->bounding_box_defined_ = true;
                 }

               }
               else
                 // no bound violations anymore - leave while loop
                 break;
             }
        }




        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief Pointer to input point cloud dataset. */
        PointCloudConstPtr input_;

        /** \brief A pointer to the vector of point indices to use. */
        IndicesConstPtr indices_;

        /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
        double epsilon_;

        /** \brief Octree resolution. */
        double resolution_;

        // Octree bounding box coordinates
        double min_x_;
        double max_x_;

        double min_y_;
        double max_y_;

        double min_z_;
        double max_z_;

        /** \brief Flag indicating if octree has defined bounding box. */
        bool bounding_box_defined_;

        /** \brief Amount of DataT objects per leafNode before expanding branch
         *  \note zero indicates a fixed/maximum depth octree structure
         * **/
        std::size_t max_objs_per_leaf_;
    };

  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_pointcloud.hpp>
#endif

#endif
