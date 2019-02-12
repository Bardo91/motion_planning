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


#ifndef MOTIONPLANNING_PLANNER_H_
#define MOTIONPLANNING_PLANNER_H_

#include <functional>

#include <motion_planning/Constraint.h>
#include <motion_planning/Trajectory.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace mp{
    /// Base class with general interface of planner
    class Planner{
    public:
        /// Add a new constraint inherited from contraint class
        void addConstraint(const Constraint _constraint);
    
        /// Set initial point
        virtual void initPoint(const Eigen::Vector3f &_initPoint);
        
        /// Set target point
        void targetPoint(const Eigen::Vector3f &_target);

        /// Compute trajectory    
        virtual Trajectory compute() = 0;

        /// Interface for enabling visualization of the algorithm while working. 
        /// This method is not guaranteed to be implemented in all the classes.
        virtual void enableDebugVisualization(std::shared_ptr<pcl::visualization::PCLVisualizer> _viewer) = 0;

    protected:
        std::vector<Constraint> constraints_;
        Eigen::Vector3f init_;
        Eigen::Vector3f target_;
    };
}

#endif