/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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

/**
   \brief
   @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <iterator>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Eigen>
#include "/home/alumno/robocomp/classes/grid2d/grid.h"
#include <complex>
#include "/home/alumno/robocomp/components/albertodiego/robotica/practica5/src/dynamic_window.h"
//#include "/home/alumno/robocomp/classes/grid2d/grid.cpp"




class SpecificWorker : public GenericWorker
{

    struct Target{
        QPointF pos;
        bool active;
    };
    struct Door{
        Eigen::Vector2f p1;
        Eigen::Vector2f p2;
        bool operator==(const Door &d1)
        {
            return (p1 - d1.p1).norm() < 700 and (p2 - d1.p2).norm() < 700 or
                                                            (p1 - d1.p2).norm() < 700 and (p2 - d1.p1).norm() < 700;
        };
        Eigen::Vector2f get_midpoint() const {return p1 + ((p2-p1)/2.0);};
        Eigen::Vector2f get_external_midpoint() const
        {
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
            return r.pointAt(1000.0);
        };


    };


Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    float gaussian(float beta);
    void update_map(const RoboCompLaser::TLaserData &ldata, RoboCompFullPoseEstimation::FullPoseEuler rState);
    void detectandoPuertas();


public slots:
    void compute();
    int startup_check();
    void initialize(int period);
    void new_target_slot(QPointF point);
private:
    std::shared_ptr < InnerModel > innerModel;
    bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;
    int numDoors=0;


    Eigen::Vector2f  world2robot(RoboCompFullPoseEstimation::FullPoseEuler &r_state, Eigen::Vector2f punto);
    Eigen::Vector2f Robot2world(RoboCompFullPoseEstimation::FullPoseEuler rState,Eigen::Vector2f tip);

    Dynamic_Window dw;
    Target target;
    Grid RobotGrid;
    std::vector<Door> vDoors;
    Door nextDoor;
    enum class State {IDLE, INIT_TURN, EXPLORING, TO_NEXT_DOOR, SEARCHING_DOOR};
    State state= State::IDLE;
};

#endif