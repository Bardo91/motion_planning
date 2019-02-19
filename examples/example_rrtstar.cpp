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
    std::cout << "Loading input cloud: ";
    auto t0 = std::chrono::system_clock::now();
    // Load cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud, filtered;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (_argv[1], cloud) == -1) {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    // Filter cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud.makeShared());
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter(filtered);

    // Draw cloud
    mp::Visualizer viz;
    viz.draw<pcl::PointXYZRGB>(filtered);  
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D (filtered, minPt, maxPt);
    auto t1 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()/1000.0 << "s." << std::endl;
    std::cout << "Configuring planner";
    // Config planner
    mp::RRTStar planner;
    planner.initPoint({minPt.x, minPt.y, minPt.z});
    planner.targetPoint({maxPt.x, maxPt.y, maxPt.z});

    // planner.enableDebugVisualization(viz.rawViewer());
    planner.iterations(20000);
    planner.dimensions( minPt.x, minPt.y, minPt.z,
                        maxPt.x, maxPt.y, /*maxPt.z*/ 5);

    // Add constraint
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(0.1);
    octree.setInputCloud(filtered.makeShared());
    octree.addPointsFromInputCloud();

    float safeDist = 1.0;
    mp::Constraint c1 = [&](const Eigen::Vector3f &_orig, const Eigen::Vector3f &_dest){
        pcl::PointXYZRGB query;
        query.x = _dest[0];
        query.y = _dest[1];
        query.z = _dest[2];
        std::vector<int> index;
        std::vector< float > dist;
        octree.nearestKSearch(query, 1, index, dist);
        
        return dist[0] > safeDist;
    };
    planner.addConstraint(c1);
    
    Eigen::Vector3f sphereCentre = {    (minPt.x + maxPt.x)/2 ,
                                        (minPt.y + maxPt.y)/2 ,
                                        /*(minPt.z + maxPt.z)/2*/ 2.0 };
    float radSphere = 20;
    mp::Constraint c2 = [&](const Eigen::Vector3f & _old, const Eigen::Vector3f &_new){
        return pow(_new[0] - sphereCentre[0], 2) + pow(_new[1] - sphereCentre[1], 2) + pow(_new[2] - sphereCentre[2], 2) - pow(radSphere,2) > 0;
    };
    planner.addConstraint(c2);
    viz.drawSphere(sphereCentre, radSphere);
    auto t2 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()/1000.0 << "s." << std::endl;
    std::cout << "Computing trajectory: ";
    // Compute traj
    auto traj = planner.compute();
    auto t3 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count()/1000.0 << "s." << std::endl;
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
    auto t4 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count()/1000.0 << "s." << std::endl;
    
    for(;;){
        viz.rawViewer()->spinOnce(30);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}
