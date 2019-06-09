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

#include <motion_planning/thirdparty/DouglasPeucker.h>
#include <motion_planning/utils/Visualization.h>

int main(int _argc, char ** _argv){
    mp::Visualizer viz;

    std::list<mp::p3d> line;
	line.push_back(mp::p3d{0,0,0});
	line.push_back(mp::p3d{1,1,2});
	line.push_back(mp::p3d{2,3,2});
	line.push_back(mp::p3d{0,1,1});
	line.push_back(mp::p3d{2,4,3});
	line.push_back(mp::p3d{4,4,4});
	line.push_back(mp::p3d{5,5,5});
	line.push_back(mp::p3d{9,7,9});
                          
    mp::Trajectory trajUnopt, trajOpt;
    for(auto &p: line){
        trajUnopt.appendPoint({std::get<0>(p), std::get<1>(p), std::get<2>(p)});
    }

    viz.draw(trajUnopt, false, 1, 255, 0, 0);

    mp::DouglasPuecker2D<mp::p3d, mp::p3dAccessor> dp3d(line);

    dp3d.simplify(atof(_argv[1]));
    std::list<mp::p3d> result = dp3d.getLine();
    for(auto &p: result){
        trajOpt.appendPoint({std::get<0>(p), std::get<1>(p), std::get<2>(p)});
    }

    viz.draw(trajOpt, false, 1);

    viz.spin();

}
