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
#include "specificworker.h"

/**
* \brief Default constructor
*/
int j=0;
int media1=0;
int media2=0;
int condition=0;
int distDer=0;
int distIzq=0;
int velocity=300;
int heading=0;
int rango=0;
int giro=2;


SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
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
<<<<<<< HEAD
    return true;
=======
	return true;
>>>>>>> baa67a17b2c16d8ab5f56a5b5cd486efec10cedb
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
        timer.start(Period);
    }

}

void SpecificWorker::compute()
{
<<<<<<< HEAD
    auto laserData = this->laser_proxy->getLaserData();
    for(auto &l: laserData) {
        if (l.dist > 800+rango && l.angle<0.8 && l.angle >-1) {
            condition=0;
        }
        if(l.dist < 1200+rango && l.angle<0.8&& l.angle >-1){
            velocity = 300;
        }
        if(l.dist > 1200+rango && l.angle<0.8&& l.angle >-1){
            velocity = 500;
        }

        if (l.dist < 800+rango && l.angle<0.5 && l.angle >-1) {
            distDer=0;
            distIzq=0;

            for(auto &l: laserData){
                if(l.angle<0){
                    distDer+=l.dist;
                    media1++;
                }else {
                    distIzq += l.dist;
                    media2++;
                }
            }
            if(distDer/media1<distIzq/media2){
                condition=2;
            }else{
                condition=1;
            }
        }

        if(rango>4000){
            condition=3;
        }
    }


    switch (condition) {
        case 0:
            recto(5,velocity);
            break;
        case 1:
            giroIzq(giro);
            break;
        case 2:
            giroDer(giro);
            break;
        case 3:
            stop();
            break;

    }

    if(heading>=420 || heading <= -420) {
        heading = 0;
        rango += 500;
        qInfo()<<"rango: "<<rango;
    }



        qInfo()<<" Heading" <<heading<< " Condition: " <<condition;
       // auto laserData = this->laser_proxy->getLaserData();
       // for(auto &l: laserData){
            //qInfo()<<l.dist<<" " <<l.angle;
        //}


=======
    try{

        auto laserData = this->laser_proxy->getLaserData();
        for(auto &l: laserData){
            qInfo()<<l.dist<<" " <<l.angle;
        }
    } catch (const Ice::AlreadyRegisteredException excp ) {
        
    }
    
>>>>>>> baa67a17b2c16d8ab5f56a5b5cd486efec10cedb
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::giroDer(int giro) {
    if(j<giro){
        this->differentialrobot_proxy->setSpeedBase(100,1);
        j++;
    }
    else{
        this->differentialrobot_proxy->setSpeedBase(0,0);
        this->differentialrobot_proxy->stopBase();
        heading +=22;
        j=0;
    }

}
void SpecificWorker::giroIzq(int giro) {
    if(j<giro){
        this->differentialrobot_proxy->setSpeedBase(100,-1);
        j++;
    }
    else{
        this->differentialrobot_proxy->setSpeedBase(0,0);
        this->differentialrobot_proxy->stopBase();
        heading-=22;
        j=0;
    }

}

void SpecificWorker::recto(int dist, int vel) {
    if(j<dist){
        this->differentialrobot_proxy->setSpeedBase(vel,0);
        j++;
    }
    else{
        this->differentialrobot_proxy->setSpeedBase(0,0);
        this->differentialrobot_proxy->stopBase();
        j=0;
    }
}


void SpecificWorker::stop() {
    this->differentialrobot_proxy->setSpeedBase(0, 0);
    this->differentialrobot_proxy->stopBase();
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData








