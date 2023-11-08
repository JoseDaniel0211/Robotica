/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/sliding_window.hpp>
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
    // Uncomment if there's too many debug messages
    // but it removes the possibility to see the messages
    // shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        // Inicializaciones personales
        viewer = new AbstractGraphicViewer(this, QRectF(-5000,-5000,10000,10000));
        viewer->add_robot(460,480,0,100,QColor("Blue"));
        viewer->show();
        viewer->activateWindow();

        timer.start(Period);
    }

}

void SpecificWorker::compute() {

    RoboCompLidar3D::TData ldata;

    ldata = lidar3d_proxy->getLidarData("bpearl", 0, 360, 1);
    qInfo() << ldata.points.size();
    const auto &points = ldata.points;
    if (points.empty()) return;

    //decltype(ldata.points) filtered_points;
    RoboCompLidar3D::TPoints filtered_points;
    std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) { return p.z < 2000; });
    //draw_lidar(filtered_points, viewer);
    tuple<RoboCompLidar3D::TPoints, RoboCompLidar3D::TPoints,RoboCompLidar3D::TPoints> tupla;
    tupla = DetectarPuertas(const_cast<RoboCompLidar3D::TPoints &>(points));
    RoboCompLidar3D::TPoints a = std::get<2>(tupla);
    draw_lidar(a, viewer);
    auto peaks = extract_peaks(tuple<RoboCompLidar3D::TPoints, RoboCompLidar3D::TPoints,RoboCompLidar3D::TPoints>);
}


int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto &p : points)
    {
        auto point = viewer->scene.addRect(-50,-50,100, 100, QPen(QColor("Blue")), QBrush(QColor("Blue")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
}

tuple<RoboCompLidar3D::TPoints, RoboCompLidar3D::TPoints,RoboCompLidar3D::TPoints> SpecificWorker::DetectarPuertas(const RoboCompLidar3D::TPoints &points){
    RoboCompLidar3D::TPoints array1, array2, array3;
    for(const auto &p : points){
        if(p.z > alturaZ1_MIN && p.z < alturaZ1_MAX){
            array1.push_back(p);
        }
        if(p.z > alturaZ2_MIN && p.z < alturaZ2_MAX){
            array2.push_back(p);
        }
        if(p.z > alturaZ3_MIN && p.z < alturaZ3_MAX){
            array3.push_back(p);
        }
    }
    return std::make_tuple(array1, array2, array3);
}
viod SpecificWorker::extract_peaks(const ){
    RoboCompLidar3D::TPoints peaks;
    for(const auto &both: iter::sliding_window(lines.low), 2)
        peaks.low[both[1].r - both[0].r]
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

