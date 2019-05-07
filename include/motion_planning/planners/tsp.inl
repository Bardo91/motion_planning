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

namespace mp{
    template<typename PointType_>
    TSP::TSP(const pcl::PointCloud<PointType_> &_points){
        for(auto &p:_points){
            pcl::PointXYZ pclP(p.x, p.y, p.z);
            points_.push_back(pclP);
        }

        graphCost_ = Eigen::MatrixXf(points_.size(),points_.size());
        nPoints_ = points_.size();

        for(unsigned i = 0; i < points_.size(); i++){
            for(unsigned j = i; j < points_.size(); j++){
                float distance = sqrt(  pow(points_[i].x - points_[j].x, 2)+
                                        pow(points_[i].y - points_[j].y, 2)+
                                        pow(points_[i].z - points_[j].z, 2)  );

                graphCost_(i, j) = distance;
                graphCost_(j, i) = distance;    
            }   
        }
    }
}