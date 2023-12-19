#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>
#include <tuple>
#include "door_detector.h"
#include <Eigen/Dense>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:
        bool startup_check_flag;
        AbstractGraphicViewer *viewer;

        struct Constants
        {
            std::string lidar_name = "helios";
            const float MAX_ADV_SPEED = 700;
            const float DOOR_PROXIMITY_THRESHOLD = 200;
            std::vector<std::pair<float, float>> ranges_list = {{1000, 2000}};
        };
        Constants consts;

        using Door = DoorDetector::Door;
        using Doors = std::vector<Door>;
        using Line = std::vector<Eigen::Vector2f>;
        using Lines = std::vector<Line>;

        // Doors
        DoorDetector door_detector;
        std::vector<Line> extract_lines(const RoboCompLidar3D::TPoints &points, const vector<std::pair<float, float>> &ranges);
        void match_door_target(const Doors &doors, const Door &target);

        // Draw
        void draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);
        void draw_target_door(const Door &target, AbstractGraphicViewer *viewer, QColor color="magenta", QColor color_far="orange");
        void draw_lines(const Lines &lines, AbstractGraphicViewer *pViewer);

        // states
        Door door_target;
        enum class States{ IDLE, SEARCH_DOOR, GOTO_DOOR, GO_THROUGH, ALIGN, CROSS_DOOR};
        States state = States::SEARCH_DOOR;
        void state_machine(const Doors &doors);

        // robot
        void move_robot(float side, float adv, float rot);
        float break_adv(float dist_to_target);
        float break_rot(float rot);


};
#endif
