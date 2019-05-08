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

#include <motion_planning/planners/tsp.h>
#include <limits>
#include <motion_planning/Trajectory.h>

namespace mp{
    TSP::TSP(const Eigen::MatrixXf &_graphCost): graphCost_(_graphCost){
        assert(_graphCost.rows() == _graphCost.cols());
        nPoints_ = _graphCost.rows();
    }

    Trajectory TSP::compute(){
        // store all vertices apart from source vertices 
        
        std::vector<int> vertices; 
        
        for (int i = 0; i < nPoints_; i++){
            if (i != initPoint_) 
                vertices.push_back(i); 
        } 
        

        // store minimum weight Hamiltonian Cycle. 
        std::vector<int> minConfiguration;
        float min_path = std::numeric_limits<float>::max(); 
        do { 
            // store current Path weight(cost) 
            float current_pathweight = 0; 
                
            // compute current path weight 
            int k = initPoint_; 
            for (int i = 0; i < vertices.size(); i++) { 
                current_pathweight += graphCost_(k,vertices[i]); 
                k = vertices[i]; 
            } 
            current_pathweight += graphCost_(k, initPoint_); 

            // update minimum 
            if( current_pathweight < min_path){
                minConfiguration = vertices;
                min_path = current_pathweight;
            }

        } while (std::next_permutation(vertices.begin(), vertices.end())); 

        std::cout << "min path: "<<min_path << std::endl;

        Trajectory traj; 
        if(points_.size() == 0){
            for(auto &v: minConfiguration){
                traj.appendPoint({(float)v,0,0});
            }
        }else{
            for(auto &v: minConfiguration){
                traj.appendPoint({points_[v].x, points_[v].y,points_[v].z});
            }
        }
        
        return traj; 
    }

}