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

#include <motion_planning/planners/rrtstar.h>

#include <thread>
#include <chrono>

#include <pcl/common/geometry.h>

namespace mp{
    //-----------------------------------------------------------------------------------------------------------------
    RRTStar::RRTStar(): octree_(stepSize_),  
                        minLimit_({-1,-1,-1}), 
                        maxLimit_({1,1,1}),
                        nodes_(new pcl::PointCloud<pcl::PointXYZ>) {
        // srand((unsigned int) time(0));
        samplerFn_ = [&](){
            Eigen::Vector3f sampledPoint = Eigen::Vector3f::Random();
            sampledPoint[0] = ((sampledPoint[0] - (-1))/2) * (maxLimit_[0] - minLimit_[0]) + minLimit_[0];
            sampledPoint[1] = ((sampledPoint[1] - (-1))/2) * (maxLimit_[1] - minLimit_[1]) + minLimit_[1];
            sampledPoint[2] = ((sampledPoint[2] - (-1))/2) * (maxLimit_[2] - minLimit_[2]) + minLimit_[2];
            return sampledPoint;
        };
    }

    //-----------------------------------------------------------------------------------------------------------------
    void RRTStar::initPoint(const Eigen::Vector3f &_initPoint){
        init_ = _initPoint;
        nodes_->push_back(pcl::PointXYZ(_initPoint[0], _initPoint[1], _initPoint[2]));
        nodesInfo_.push_back({0, -1, 0.0f});
        octree_.setInputCloud(nodes_);
        octree_.addPointsFromInputCloud();

        if(viewer_){
            viewer_->addPointCloud(nodes_, "nodes");
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    Trajectory RRTStar::compute(){
        for(unsigned iter = 0; iter < iterations_ ; iter++){
            auto newPoint = sampleFree();
            auto nearestPointId = nearest(newPoint);
            auto nearestPoint = Eigen::Vector3f(nodes_->points[nearestPointId].x, 
                                                nodes_->points[nearestPointId].y, 
                                                nodes_->points[nearestPointId].z);
            newPoint = steer(newPoint, nearestPoint);
            
            if(checkConstraints(nearestPoint, newPoint)){
                // Add point
                auto newId = addPoint(newPoint);

                auto neighbors = neighborhood(newId, neighborSearchDistance_);

                // Check connections
                auto xminId = nearestPointId;
                auto cmin = costFromOrigin(nearestPointId) + (nearestPoint - newPoint).norm();
                for(auto &xId:neighbors){
                    if(xId == newId)
                        continue;

                    auto neighbor = Eigen::Vector3f(nodes_->points[xId].x, 
                                                    nodes_->points[xId].y, 
                                                    nodes_->points[xId].z);
                    auto c = costFromOrigin(xId) + (neighbor- newPoint).norm();
                    // std::cout << newPoint.transpose() << " | " << neighbor.transpose() << " | " << costFromOrigin(xId) << " | " << (neighbor- newPoint).norm() << std::endl;
                    if(c < cmin && checkConstraints(neighbor, newPoint)){
                        xminId = xId;
                        cmin = c;
                    }
                }

                // Add edge info
                nodesInfo_[newId].parent_ = xminId;
                nodesInfo_[newId].cost_ = cmin;

                if(viewer_){
                    viewer_->addLine(nodes_->points[newId], nodes_->points[xminId], "line_"+std::to_string(newId)+std::to_string(xminId));
                    // drawNodesTree_->InsertNextPoint(    newPoint[0], 
                    //                                     newPoint[1], 
                    //                                     newPoint[2]);
                    // vtkIdType connectivity[2];
                    // connectivity[0] = xminId;
                    // connectivity[1] = newId;
                    // drawTree_->InsertNextCell(VTK_LINE,2,connectivity);
                }

                // Rewiring
                for(auto &xId:neighbors){
                    if(xId == newId)
                        continue;
                        
                    auto neighbor = Eigen::Vector3f(nodes_->points[xId].x, 
                                                    nodes_->points[xId].y, 
                                                    nodes_->points[xId].z);
                    auto cnew = costFromOrigin(newId) + (neighbor- newPoint).norm();
                    auto cnear = costFromOrigin(xId);
                    if(cnew < cnear && checkConstraints(neighbor, newPoint)){
                        if(viewer_){
                            viewer_->removeShape("line_"+std::to_string(newId)+std::to_string(xminId));
                            viewer_->addLine(nodes_->points[newId], nodes_->points[xId], "line_"+std::to_string(newId)+std::to_string(xminId));
                        }
                        nodesInfo_[xId].parent_ = newId;
                    }
                }

                if(viewer_){
                    viewer_->spinOnce();
                }
                // std::cout << "----------------" << std::endl;
            }
        }
        
        Trajectory traj;

        int id = nearest(target_);
        do{
            Eigen::Vector3f p(nodes_->points[id].x,nodes_->points[id].y, nodes_->points[id].z);
            traj.appendPoint(p);
            id = nodesInfo_[id].parent_;
        }while(id != -1);
        
        return traj;
    }

    //-----------------------------------------------------------------------------------------------------------------
    void RRTStar::iterations(int _iters){
        iterations_ = _iters;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int RRTStar::iterations(){
        return iterations_;
    }

    //-----------------------------------------------------------------------------------------------------------------
    void RRTStar::stepSize(float _step){
        stepSize_ = _step;
        neighborSearchDistance_ = _step*1.1;
        octree_.setResolution(stepSize_);
    }

    //-----------------------------------------------------------------------------------------------------------------
    void RRTStar::enableDebugVisualization(std::shared_ptr<pcl::visualization::PCLVisualizer> _viewer){
        viewer_ = _viewer;

        // drawTree_ = vtkSmartPointer<vtkPolyData>::New();
        // drawTree_->Allocate();
        // drawNodesTree_ = vtkSmartPointer<vtkPoints>::New();
        // drawTree_->SetPoints(drawNodesTree_);
        // viewer_->addModelFromPolyData(drawTree_, "tree");

    }

    //-----------------------------------------------------------------------------------------------------------------
    void RRTStar::dimensions(float _xmin, float _ymin, float _zmin, float _xmax, float _ymax, float _zmax){
        minLimit_ = {_xmin, _ymin, _zmin};
        maxLimit_ = {_xmax, _ymax, _zmax};
    }

    //-----------------------------------------------------------------------------------------------------------------
    void RRTStar::samplerFunction(std::function<Eigen::Vector3f(void)> _samplerFn){
        samplerFn_ = _samplerFn;
    }

    //-----------------------------------------------------------------------------------------------------------------
    void RRTStar::tree(pcl::PointCloud<pcl::PointXYZ>::Ptr &_nodes, std::vector<NodeInfo>& _nodesInfo){
        _nodes = nodes_;
        _nodesInfo = nodesInfo_;
    }

    //-----------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f RRTStar::sampleFree(){
        return samplerFn_();
    }

    //-----------------------------------------------------------------------------------------------------------------
    int RRTStar::nearest(const Eigen::Vector3f &_point){
        pcl::PointXYZ query(_point[0], _point[1], _point[2]);
        std::vector<int> index;
        std::vector< float > dist;
        octree_.nearestKSearch(query, 1, index, dist);
        return index[0];
    }

    //-----------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f RRTStar::steer(const Eigen::Vector3f &_newPoint, const Eigen::Vector3f &_nearestPoint){
        Eigen::Vector3f dir = _newPoint - _nearestPoint;
        auto norm = dir.norm();
        if(norm < stepSize_){
            return _newPoint;
        }else{
            dir /= norm;
            return _nearestPoint + dir*stepSize_;
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool RRTStar::checkConstraints(const Eigen::Vector3f &_orig, const Eigen::Vector3f &_dest){
        for(auto &constraint: constraints_){
            if(!constraint(_orig, _dest))
                return false;
        }
        return true;
    }  

    //-----------------------------------------------------------------------------------------------------------------
    int RRTStar::addPoint(const Eigen::Vector3f &_point){
        int id = nodes_->size();

        pcl::PointXYZ pclPoint(_point[0], _point[1], _point[2]);

        nodesInfo_.push_back({id, -1, 0.0f});

        octree_.addPointToCloud(pclPoint, nodes_);

        if(viewer_){
            viewer_->updatePointCloud(nodes_, "nodes");
        }

        return id;
    }

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<int> RRTStar::neighborhood(int _id, float _distance){
        std::vector<int> indices;
        std::vector<float> distances;
        octree_.radiusSearch(   _id,
                                _distance,
                                indices, 
                                distances);
        return indices;
    }

    //-----------------------------------------------------------------------------------------------------------------
    float RRTStar::costFromOrigin(int _vertexId){
        float totalCost = 0.0f;

        int id = _vertexId;
        while(id > 0){
            Eigen::Vector3f p1 = {nodes_->points[id].x, nodes_->points[id].y, nodes_->points[id].z};
            Eigen::Vector3f p2 = {nodes_->points[nodesInfo_[id].parent_].x, nodes_->points[nodesInfo_[id].parent_].y, nodes_->points[nodesInfo_[id].parent_].z};
            totalCost += (p2-p1).norm();
            id = nodesInfo_[id].parent_;
        }

        return totalCost;
    }

}