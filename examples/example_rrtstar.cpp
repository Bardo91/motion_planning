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

#include <thread>
#include <chrono>

int main(int _argc, char** _argv){    
    mp::Visualizer viz;
/*
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (_argv[1], cloud) == -1) {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    viz.draw<pcl::PointXYZ>(cloud);
*/

    mp::RRTStar planner;

    planner.initPoint({-0.5,-0.5,-0.5});
    planner.targetPoint({1,1,1});

    planner.enableDebugVisualization(viz.rawViewer());
    planner.iterations(1000);
    // planner.dimensions(-5,-5,-5,5,5,5);
    
    Eigen::Vector3f sphereCentre = {0.3,0.3,0.3};
    float radSphere = 0.5;
    mp::Constraint c1 = [&](const Eigen::Vector3f & _old, const Eigen::Vector3f &_new){
        return pow(_new[0] - sphereCentre[0], 2) + pow(_new[1] - sphereCentre[1], 2) + pow(_new[2] - sphereCentre[2], 2) - pow(radSphere,2) > 0;
    };
    planner.addConstraint(c1);

    auto traj = planner.compute();

    viz.drawSphere(sphereCentre, radSphere);
    viz.draw(traj);

    for(;;){
        viz.rawViewer()->spinOnce(30);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}
