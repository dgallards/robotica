#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
#include <cppitertools/enumerate.hpp>
#include "specificworker.h"

float MAX_ADV_SPEED = 1000;
float MAX_LASER_DIST=4000;
float tile_size=100.0;
float semiancho = 100;
int condition=0;
int veces = 5;
double initial_angle;


/**
* \brief Default constructor
*/
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
    return true;
}

void SpecificWorker::initialize(int period)
{
    target.active=false;
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
    QRect dimensions(-5000,-2500,10000,5000);
    viewer = new AbstractGraphicViewer(this, dimensions);
    RobotGrid.initialize(dimensions,50,&viewer->scene, false);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
}

void SpecificWorker::compute()
{
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    RoboCompLaser::TLaserData  lData;
    try{
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x,r_state.y);
        lData = laser_proxy->getLaserData();
        draw_laser(lData);
    }catch(const Ice::Exception &e){ std::cout << e.what()<<std::endl;}

    update_map(lData,r_state);

    float rot_speed = 0;
    float initial_angle;
    static std::vector<Eigen::Vector2f> path;

    switch (state){
        case State::IDLE:
            state = State::INIT_TURN;
            break;
        case State::INIT_TURN:
            initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;

            state = State::EXPLORING;
            break;
        case State::EXPLORING:
        {
            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            if (fabs(current - initial_angle) < (M_PI + 0.1) and fabs(current - initial_angle) > (M_PI - 0.1)) {
                state = State::SEARCHING_DOOR;
            } else {
                rot_speed = 0.4;
                differentialrobot_proxy->setSpeedBase(0, rot_speed);

                detectandoPuertas();

            }
            break;
        }
        case State::SEARCHING_DOOR:
            if(vDoors.size()>0){
                nextDoor = vDoors[vDoors.size()-1];
            }
            path.push_back( nextDoor.get_external_midpoint());
            state = State::TO_NEXT_DOOR;
            break;
        case State::TO_NEXT_DOOR:
        {
            auto middle_point = path.back();
            auto tr = world2robot(r_state,middle_point);
            auto dist = tr.norm();
            if(dist > 150){
                QPolygonF laser_poly;
                for(auto &&l : lData)
                    laser_poly<<QPointF(l.dist*sin(l.angle),l.dist*cos(l.angle));
                auto [_, __, adv, rot, ___] = dw.compute(tr, laser_poly,
                                                         Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                         Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                         nullptr /*&viewer_robot->scene*/);
                const float rgain = 0.8;
                float rotation = rgain*rot;
                float dist_break = std::clamp(world2robot(r_state,tr).norm()/1000.0, 0.0,1.0);
                float advance = MAX_ADV_SPEED * dist_break * gaussian(rotation);
                differentialrobot_proxy->setSpeedBase(advance,rotation);
            }
            else{
                differentialrobot_proxy->setSpeedBase(0,0);
                state = State::EXPLORING;
            }
            break;
        }

    }

}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{

    static QGraphicsItem *laser_polygon = nullptr;
    if(laser_polygon!= nullptr)
        viewer->scene.removeItem(laser_polygon);

    QPolygonF poly;

    poly << QPointF(0,0);
    double x, y;
    for(auto &lData : ldata){
        x = lData.dist * sin(lData.angle);
        y = lData.dist * cos(lData.angle);
        poly << QPointF(x,y);
    }

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::new_target_slot(QPointF point){
    target.pos= point;
    qInfo() << point << endl;
    target.active=true;

}
float SpecificWorker::gaussian(float beta)
{
    return (1/sqrt(2*3.141632))* exp(-0.5*pow(beta-0.1,2));

}

Eigen::Vector2f SpecificWorker::world2robot(RoboCompFullPoseEstimation::FullPoseEuler &r_state, Eigen::Vector2f punto)
{
    float angle = r_state.rz;
    Eigen::Vector2f T(r_state.x, r_state.y), point_in_world = punto;
    Eigen::Matrix2f R;
    R <<  cos(angle), sin(angle),
            -sin(angle), cos(angle);
    Eigen::Vector2f point_in_robot = R * (point_in_world - T);
    return point_in_robot;
}

Eigen::Vector2f SpecificWorker::Robot2world(RoboCompFullPoseEstimation::FullPoseEuler rState,Eigen::Vector2f tip){
    float angle = rState.rz;
    Eigen::Vector2f T(rState.x, rState.y);
    Eigen::Vector2f point_in_robot(tip.x(), tip.y());
    Eigen::Matrix2f R;
    R << cos(angle), -sin(angle), sin(angle), cos(angle);
    Eigen::Vector2f point_in_world = R * point_in_robot + T;
    return point_in_world;
}

void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata, RoboCompFullPoseEstimation::FullPoseEuler rState) {
        Eigen::Vector2f lw;
        for(const auto &l : ldata)
        {
            if(l.dist > semiancho)
            {
                Eigen::Vector2f tip(l.dist*sin(l.angle), l.dist*cos(l.angle));
                Eigen::Vector2f p = Robot2world(rState,tip);
                int target_kx = (p.x() - RobotGrid.dim.left()) / RobotGrid.TILE_SIZE;
                int target_kz = (p.y() - RobotGrid.dim.bottom()) / RobotGrid.TILE_SIZE;
                int last_kx = -1000000;
                int last_kz = -1000000;

                int num_steps = ceil(l.dist/(tile_size/2.0));
                for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
                {
                    Eigen::Vector2f p = Robot2world(rState,tip*step);
                    int kx = (p.x() - RobotGrid.dim.left()) / RobotGrid.TILE_SIZE;
                    int kz = (p.y() - RobotGrid.dim.bottom()) / RobotGrid.TILE_SIZE;
                    if(kx != last_kx and kx != target_kx and kz != last_kz and kz != target_kz)
                        RobotGrid.add_miss(Robot2world(rState,tip * step));
                    last_kx = kx;
                    last_kz = kz;
                }
                if(l.dist <= MAX_LASER_DIST)
                    RobotGrid.add_hit(Robot2world(rState,tip));
                else
                    RobotGrid.add_miss(Robot2world(rState,tip));
            }
        }
}

void SpecificWorker::detectandoPuertas(){
    auto r_state = fullposeestimation_proxy->getFullPoseEuler();
    auto lData = laser_proxy->getLaserData();

    std::vector<float> derivatives(lData.size());
    derivatives[0] = 0;
    for (const auto &&[k, l]: iter::sliding_window(lData, 2) | iter::enumerate)
        derivatives[k + 1] = l[1].dist - l[0].dist;

    std::vector<Eigen::Vector2f> peaks;
    for (const auto &&[k, der]: iter::enumerate(derivatives))
    {
        RoboCompLaser::TData l;
        if (der > 600)
        {
            l = lData.at(k - 1);
            peaks.push_back(Robot2world(r_state,Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
        }
        else if (der < -600)
        {
            l = lData.at(k);
            peaks.push_back(Robot2world(r_state,Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
        }
    }
    for (auto&& c : iter::combinations_with_replacement(peaks, 2))
    {

        if((c[0]-c[1]).norm() < 1200 and (c[0]-c[1]).norm() > 700){
            Door door{c[0],c[1]};
            static std::vector<QGraphicsItem*> door_line;

            if(auto r = std::ranges::find_if(vDoors,[door](auto d){return d == door; }); r == vDoors.end())
                vDoors.push_back(door);

            for(const auto p: vDoors)
            {

                door_line.push_back(viewer->scene.addLine(QLine(p.p1.x(), p.p1.y(), p.p2.x(), p.p2.y()),
                                                            QPen(QColor("Blue"),100)));
                door_line.back()->setZValue(200);
            }

        }
    }
}








