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

#include <motion_planning/Trajectory.h>

namespace mp{
        //-------------------------------------------------------------------------------------------------------------
        void Trajectory::appendPoint(const Eigen::Vector3f &_point){
                points_.push_back(_point);
        }

        //-------------------------------------------------------------------------------------------------------------
        std::vector<Eigen::Vector3f> Trajectory::points() const{
                return points_;
        }
        
        //-------------------------------------------------------------------------------------------------------------
        float Trajectory::distance() const{
                float distance = 0;
                for(unsigned i = 1; i < points_.size(); i++){
                        distance += sqrt(
                                pow(points_[i-1][0] - points_[i][0], 2)+
                                pow(points_[i-1][1] - points_[i][1], 2)+
                                pow(points_[i-1][2] - points_[i][2], 2)
                        );
                }
                return distance;
        }

        //-------------------------------------------------------------------------------------------------------------
        std::ostream& operator<<(std::ostream& os, const Trajectory& _traj) {
                for(unsigned i = 0; i < _traj.points_.size(); i++){
                        auto p = _traj.points_[i];
                        os << "("<<i<<")("<<p[0]<<","<<p[1]<<","<<p[2]<<")\n";
                }
                return os;
        }

}