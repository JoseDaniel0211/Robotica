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
    try
    {
        ldata = lidar3d_proxy->getLidarData("bpearl", 0, 360, 1);
        qInfo() << ldata.points.size();
        const auto &points = ldata.points;
        if (points.empty()) return;


    //decltype(ldata.points) filtered_points;
    RoboCompLidar3D::TPoints filtered_points;
    std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) { return p.z < 2000;});
    draw_lidar(filtered_points, viewer);
    std::tuple<Estado, RobotSpeed> res;
    switch(estado)
    {
        case Estado::IDLE:
            stop();
            break;
        case Estado::FOLLOW_WALL:
            follow_wall(const_cast<RoboCompLidar3D::TPoints &>(points));
            break;
        case Estado::STRAIGHT_LINE: {
            chocachoca(const_cast<RoboCompLidar3D::TPoints &>(points));

            break;
        }
        case Estado::SPIRAL:
            break;
    }
    }
    catch (const Ice::Exception &e)
    {
        std::cout << "Error reading from Camera" << e << std::endl;
    }
    /*
    estado = std::get<0>(res);
    auto &[estado_, robotspeed_] = res;
    estado = estado_;
    */
    try
    {
        //omnirobot_proxy->setSpeedBase(0,0,0.5);
    }
    catch(const Ice::Exception &e) {
        std::cout << "Error reading from Camera" << e << std::endl;
    }
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

std::tuple<SpecificWorker::Estado , SpecificWorker::RobotSpeed> SpecificWorker::chocachoca(RoboCompLidar3D::TPoints &points) {
    int offset = points.size()/2-points.size()/3;
    auto min_elem = std::min_element(points.begin()+offset, points.end()-offset, [](auto  a, auto b)
    { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    RobotSpeed robot_speed;
    const float MIN_DISTANCE = 1000;
    qInfo() << std::hypot(min_elem->x, min_elem->y);
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        robot_speed = RobotSpeed{ .adv=0, .side=0, .rot=0.5};
        omnirobot_proxy->setSpeedBase(0, 0, 0.5);
    }else{

        RobotSpeed robot_speed{ .adv = 1000/1000.f, .side = 0, .rot = 0 };
        qInfo() << "Velocidad: adv=" << robot_speed.adv << ", side=" << robot_speed.side << ", rot=" << robot_speed.rot;
        omnirobot_proxy->setSpeedBase(robot_speed.adv, robot_speed.side, robot_speed.rot);
        robot_speed = RobotSpeed{ .adv=1000/1000.f, .side=0, .rot=0};
    }
    return std::make_tuple(Estado::STRAIGHT_LINE, robot_speed);
}
/*
std::tuple<SpecificWorker::Estado, SpecificWorker::RobotSpeed> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points) {
    // Parámetros de control
    const float DISTANCIA_OBJETIVO = 500; // Distancia objetivo a la pared en milímetros
    const float VELOCIDAD_AVANCE = 0.5;   // Velocidad de avance constante
    const float GANANCIA_P = 0.05;         // Ganancia proporcional para el control

    int offset = points.size() / 2 - points.size() / 3;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset, [](auto a, auto b) {
        return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
    });
    RobotSpeed robot_speed;
    if ( min_elem->y < DISTANCIA_OBJETIVO) {
        float distancia_actual = std::hypot(min_elem->x, min_elem->y);
        qInfo() << "Distancia a la pared: " << distancia_actual;

        // Control proporcional para mantener la distancia a la pared
        float error = DISTANCIA_OBJETIVO - distancia_actual;
        float velocidad_rotacion = GANANCIA_P * error;

        // Establecer la velocidad del robot

        omnirobot_proxy->setSpeedBase(0, 0, velocidad_rotacion);
    }
    else{

        omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);
    }
    return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);
}
*/
std::tuple<SpecificWorker::Estado , SpecificWorker::RobotSpeed> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &points) {
    int offset = points.size() / 2 - points.size() / 3;
    auto min_elem = std::min_element(points.begin() + offset, points.end() - offset, [](auto a, auto b)
    { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    const float MIN_DISTANCE = 495;
    //const float MIN_DISTANCE_X =
    qInfo() <<"x: "<< abs(min_elem->x)<<"y: "<< abs(min_elem->y);
    RobotSpeed robot_speed;

    if ( abs(min_elem->y) < MIN_DISTANCE) {
        qInfo() << "aaaaaaa";
        omnirobot_proxy->setSpeedBase(0, 0, 1);
        if(abs(min_elem->x) > MIN_DISTANCE){
            qInfo() << "bbbbb";
            omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);
        }


    } else {
        qInfo() << "hola";
        omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);

        // Establece la velocidad del robot para seguir la pared.
        RobotSpeed robot_speed{ .adv = 1000 / 1000.f, .side = 0, .rot = 0 };
        omnirobot_proxy->setSpeedBase(robot_speed.adv, robot_speed.side, robot_speed.rot);
        robot_speed = RobotSpeed{ .adv=1000/1000.f, .side=0, .rot=0};
    }
    return std::make_tuple(Estado::FOLLOW_WALL, robot_speed);
}

/*
std::tuple<SpecificWorker::Estado , SpecificWorker::RobotSpeed> SpecificWorker::spiral() {
    const float SPIRAL_INCREMENT = 0.1;  // Valor de incremento del radio de la espiral
    float spiralRadius = 0.0;  // Radio inicial de la espiral
    float rotationSpeed = 0.5;  // Velocidad de rotación constante

    RobotSpeed robot_speed;
    while (true) {
        // Calcula la posición en la espiral (coordenadas polares).
        float x = spiralRadius * std::cos(spiralRadius);
        float y = spiralRadius * std::sin(spiralRadius);

        // Establece la velocidad del robot para moverse en la espiral.
        robot_speed = RobotSpeed{ .adv = 1000 / 1000.f, .side = 0, .rot = rotationSpeed };

        // Actualiza el radio de la espiral para avanzar en la espiral.
        spiralRadius += SPIRAL_INCREMENT;

        // Aplica una pausa para controlar la velocidad de la espiral.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return std::make_tuple(Estado::SPIRAL, robot_speed);
}
*/
std::tuple<SpecificWorker::Estado , SpecificWorker::RobotSpeed> SpecificWorker::stop() {
    omnirobot_proxy->setSpeedBase(0, 0, 0);
    RobotSpeed robot_speed;
    robot_speed = RobotSpeed{ .adv=0, .side=0, .rot=0};
    return std::make_tuple(Estado::IDLE, robot_speed);
}




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

