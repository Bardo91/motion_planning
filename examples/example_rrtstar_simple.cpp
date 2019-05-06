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

#include <motion_planning/utils/Visualization.h>
#include <motion_planning/Trajectory.h>

#include <motion_planning/planners/rrtstar.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <chrono>


int main(int _argc, char** _argv){    
    if(_argc < 3 ){
        std::cout << "Bad inputs, introduce step size and iterations" << std::endl;
        return -1;
    }
    mp::Visualizer viz;
    std::cout << "Configuring planner";
    // Config planner
    mp::RRTStar planner(atof(_argv[1]));
    
    planner.initPoint({0,0,0});
    planner.targetPoint({1,1,1});

    // planner.enableDebugVisualization(viz.rawViewer());
    planner.iterations(atoi(_argv[2]));
    planner.dimensions( 0,0,0,
                        1,1,1);


    std::cout << "Computing trajectory: ";
    // Compute traj
    auto traj = planner.compute();
    std::cout << "Display results: ";
    std::vector<mp::RRTStar::NodeInfo> nodesInfo;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nodes;
    planner.tree(nodes, nodesInfo);

    vtkSmartPointer<vtkPolyData> treeBase;
    viz.drawCustom([&](std::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer){
        // Create new graph
        treeBase = vtkSmartPointer<vtkPolyData>::New();
        treeBase->Allocate();
        vtkSmartPointer<vtkPoints> covisibilityNodes = vtkSmartPointer<vtkPoints>::New();

        // Fill-up with nodes
        for(unsigned i = 0; i <  nodes->size(); i++){
            covisibilityNodes->InsertNextPoint(     nodes->points[i].x, 
                                                    nodes->points[i].y, 
                                                    nodes->points[i].z);
            if(i > 0){
                vtkIdType connectivity[2];
                connectivity[0] = nodesInfo[i].id_;
                connectivity[1] = nodesInfo[i].parent_;
                treeBase->InsertNextCell(VTK_LINE,2,connectivity);
            }
        }

        treeBase->SetPoints(covisibilityNodes);
        _viewer->addModelFromPolyData(treeBase, "treeRRT");
    });

    // Draw result.
    viz.draw(traj, true);
    
    for(;;){
        viz.rawViewer()->spinOnce(30);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}
