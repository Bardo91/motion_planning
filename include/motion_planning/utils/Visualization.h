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

#ifndef MOTIONPLANNING_VISUALIZATION_H_
#define MOTIONPLANNING_VISUALIZATION_H_

#include <pcl/visualization/pcl_visualizer.h>

#include <motion_planning/Trajectory.h>

namespace mp{

    class Visualizer{
    public:
        /// Basic constructor
        Visualizer();

        /// Draw a trajectory
        void draw(const Trajectory &_trajectory);

        /// Draw point cloud 
        template<typename PointType_>
        void draw(const pcl::PointCloud<PointType_> &_mesh, const Eigen::Matrix4f &_pose = Eigen::Matrix4f::Identity());

        /// Draw polygon mesh 
        void draw(const pcl::PolygonMesh &_mesh, const Eigen::Matrix4f &_pose);

        /// Draw polygon mesh 
        void drawSphere(const Eigen::Vector3f &_center, float _radius);

        /// block visualizer to render
        void spin();

        /// Non-blocking spin to visualizer for rendering
        void spinOnce();

        /// Get raw viewer. Please use it carefully
        std::shared_ptr<pcl::visualization::PCLVisualizer>  rawViewer();
        
    private:
        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
        int itemCounter_ = 0;

        std::vector<vtkSmartPointer<vtkPolyData>> trajectories_;
    };
}


#include <motion_planning/utils/Visualization.hpp>


#endif