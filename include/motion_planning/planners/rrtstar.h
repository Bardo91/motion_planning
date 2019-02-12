//---------------------------------------------------------------------------------------------------------------------
//  MOTION_PLANNING
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#ifndef MOTIONPLANNING_PLANNERS_RRTSTAR_H_
#define MOTIONPLANNING_PLANNERS_RRTSTAR_H_

#include <motion_planning/Planner.h>
#include <pcl/octree/octree_search.h>

namespace mp{
    /// Base class with general interface of planner
    class RRTStar: public Planner{
    public:
        /// Basic constructor
        RRTStar();

        /// Set initial point
        virtual void initPoint(const Eigen::Vector3f &_initPoint) override;
        
        /// Compute trajectory    
        virtual Trajectory compute();

        /// Set number of iteratios for the sampling process
        void iterations(int _iters);

        /// Get number of iterations
        int iterations();

        /// Set maximum step size for the tree
        void stepSize(float _step);

        /// Set radious search size for rewiring
        void neighborhoodSize(float _nSize);

        /// Interface for enabling visualization of the algorithm while working. 
        virtual void enableDebugVisualization(std::shared_ptr<pcl::visualization::PCLVisualizer> _viewer);

        /// Set dimensios for the sampler
        void dimensions(float _xmin, float _ymin, float _zmin, float _xmax, float _ymax, float _zmax);

    private:
        Eigen::Vector3f sampleFree();
        int nearest(const Eigen::Vector3f &_point);
        Eigen::Vector3f steer(const Eigen::Vector3f &_newPoint, const Eigen::Vector3f &_nearestPoint);
        bool checkConstraints(const Eigen::Vector3f &_orig, const Eigen::Vector3f &_dest);
        int addPoint(const Eigen::Vector3f &_point);

        std::vector<int> neighborhood(int _id, float _distance);
        float costFromOrigin(int _vertexId);

    private:
        int iterations_ = 100;
        float stepSize_ = 0.1;
        float neighborSearchDistance_ = 0.15;
        struct NodeInfo{
            int id_;
            int parent_;
            float  cost_; // From id to parent
        };

        std::vector<NodeInfo> nodesInfo_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr nodes_;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;

        Eigen::Vector3f minLimit_, maxLimit_;

        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
        vtkSmartPointer<vtkPolyData> drawTree_;
        vtkSmartPointer<vtkPoints> drawNodesTree_;
    };
}

#endif