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

#ifndef MOTIONPLANNING_TRAJECTORY_H_
#define MOTIONPLANNING_TRAJECTORY_H_

#include <Eigen/Eigen>
#include <vector>
#include <iostream>

namespace mp{
    /// Class that represents the resulting trajectory of a planner
    class Trajectory{
    public:
        /// Append point to trajectory
        void appendPoint(const Eigen::Vector3f &_point);

        /// get list of points in trajectory
        std::vector<Eigen::Vector3f> points() const;

        friend std::ostream& operator<<(std::ostream& os, const Trajectory& _traj);
    private:
        std::vector<Eigen::Vector3f> points_;
        
    };
}

#endif