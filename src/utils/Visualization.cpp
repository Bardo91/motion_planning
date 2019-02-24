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

#include <motion_planning/utils/splines.hpp>

namespace mp{
    //-------------------------------------------------------------------------------------------------------------
    Visualizer::Visualizer(){
        viewer_ = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("motion_planner_visualizer"));
        viewer_->addCoordinateSystem(0.5);
    }

    //-------------------------------------------------------------------------------------------------------------
    void Visualizer::draw(const Trajectory &_trajectory, bool _useSpline){
        // Create new graph
        vtkSmartPointer<vtkPolyData> covisibilityGraph = vtkSmartPointer<vtkPolyData>::New();
        covisibilityGraph->Allocate();
        trajectories_.push_back(covisibilityGraph);
    
        vtkSmartPointer<vtkUnsignedCharArray> covisibilityNodeColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        covisibilityNodeColors->SetNumberOfComponents(3);
        covisibilityNodeColors->SetName("Colors");

        vtkSmartPointer<vtkPoints> covisibilityNodes = vtkSmartPointer<vtkPoints>::New();

        // Fill-up with nodes
        auto points = _trajectory.points();
        
        if(_useSpline){
            Spline<Eigen::Vector3f, float> spl(5);
            spl.set_ctrl_points(points);
            int nPoints = points.size()*10;
            for(unsigned i = 0; i < nPoints; i++){
                auto p = spl.eval_f(1.0 / nPoints* i );
                const unsigned char green[3] = {0, 255, 0};
                covisibilityNodes->InsertNextPoint(     p[0], 
                                                        p[1], 
                                                        p[2]);
                covisibilityNodeColors->InsertNextTupleValue(green);
                if(i > 0){
                    vtkIdType connectivity[2];
                    connectivity[0] = i-1;
                    connectivity[1] = i;
                    covisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity);
                }
            }
        }else{
            for(unsigned i = 0; i <  points.size(); i++){
                const unsigned char green[3] = {0, 255, 0};
                covisibilityNodes->InsertNextPoint(    points[i][0], 
                                                        points[i][1], 
                                                        points[i][2]);
                covisibilityNodeColors->InsertNextTupleValue(green);
                if(i > 0){
                    vtkIdType connectivity[2];
                    connectivity[0] = i-1;
                    connectivity[1] = i;
                    covisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity);
                }
            }            
        }
        covisibilityGraph->SetPoints(covisibilityNodes);
        covisibilityGraph->GetPointData()->SetScalars(covisibilityNodeColors);
        
        viewer_->addModelFromPolyData(covisibilityGraph, "traj_"+std::to_string(itemCounter_));
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "traj_"+std::to_string(itemCounter_));
        
        itemCounter_++;
    }

    //-------------------------------------------------------------------------------------------------------------
    void Visualizer::draw(const pcl::PolygonMesh &_mesh, const Eigen::Matrix4f &_pose){
        std::string itemName = "polygon_"+std::to_string(itemCounter_);
        viewer_->addPolygonMesh(_mesh,itemName);

        itemCounter_++;
    }

    //-------------------------------------------------------------------------------------------------------------
    void Visualizer::drawSphere(const Eigen::Vector3f &_center, float _radius){
        std::string itemName = "sphere_"+std::to_string(itemCounter_);
        viewer_->addSphere(pcl::PointXYZ(_center[0], _center[1], _center[2]), _radius, itemName);

        itemCounter_++;
    } 

    //-------------------------------------------------------------------------------------------------------------
    void Visualizer::drawCustom(std::function<void(std::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer)> _func){
        _func(viewer_);
    }

    //-------------------------------------------------------------------------------------------------------------
    void Visualizer::spin(){
        viewer_->spin();
    }

    //-------------------------------------------------------------------------------------------------------------
    void Visualizer::spinOnce(){
        viewer_->spinOnce();
    }

    //-------------------------------------------------------------------------------------------------------------
    std::shared_ptr<pcl::visualization::PCLVisualizer>  Visualizer::rawViewer(){
        return viewer_;
    }
        
}
