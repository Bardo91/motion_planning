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
#include <motion_planning/utils/Visualization.h>
#include <motion_planning/Trajectory.h>

#include <motion_planning/planners/rrtstar.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <motion_planning/thirdparty/DouglasPeucker.h>

#include <thread>
#include <chrono>

  
// driver program to test above function 
int main(int _argc, char **_argv)  { 

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

    bool isFirstClick = true;
    pcl::PointXYZ lastPoint;

    viz.attachPointClickEvent([&](const pcl::visualization::PointPickingEvent &_event){
        pcl::PointXYZ newPoint;
        _event.getPoint(newPoint.x, newPoint.y, newPoint.z);
        if(isFirstClick){
            isFirstClick = false;
            lastPoint = newPoint;
        }else{
            viz.drawCustom([&](std::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer){
                auto lineName = std::to_string(time(NULL));
                _viewer->addLine(lastPoint, newPoint, 0,1,0, lineName);
                _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, lineName);
                lastPoint = newPoint;
            });
        }
    });

    
    viz.spin();
} 