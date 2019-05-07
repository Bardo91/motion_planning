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


#ifndef MOTIONPLANNING_PLANNERS_TSP_H_
#define MOTIONPLANNING_PLANNERS_TSP_H_

#include <motion_planning/Planner.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mp{
    /// Naive implementation of TSP problem solver by computing all permutations
    class TSP : public Planner{
    public:
        /// Construct problem using directly the graph costs
        TSP(const Eigen::MatrixXf &_graphCost);

        /// Construct problem using a set of points
        template<typename PointType_>
        TSP(const pcl::PointCloud<PointType_> &_points);

        /// Compute trajectory    
        virtual Trajectory compute();

    private:
        Eigen::MatrixXf graphCost_;
        pcl::PointCloud<pcl::PointXYZ> points_;
        int initPoint_ = 0;
        int nPoints_ = 0;
    };
}

#include <motion_planning/planners/tsp.inl>

#endif