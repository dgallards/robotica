#include "specificworker.h"

float MAX_ADV_SPEED = 1000;
float semiancho = 100;
int condition=0;
int veces = 5;
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
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    robot_polygon->setRotation(bState.alpha*180/M_PI);
    robot_polygon->setPos(bState.x, bState.z);
    auto lData = laser_proxy->getLaserData();
    draw_laser(lData,bState);
    condition=1;
    if(target.active){
        auto [x,y] = world2robot(bState);
        float beta = atan2(x,y);
        float dist = sqrt(pow(x,2) + pow(y,2));
        float rot_speed = beta;
        float s = gaussian(beta);
        float adv_speed;

        //Maquina de estado
        for(auto &l : lData) {
            if (l.dist < 900 && l.angle < 0.8 && l.angle > -1) {
                condition = 2;
                break;
            }
            else{
                condition = 1;
            }
        }
        if(dist<200)
            condition=0;

        //Switch de la maquina de estado
        try {
            switch (condition){
                case 0:
                    //Ha llegado al destino
                    dist = 0;
                    adv_speed = 0;
                    rot_speed = 0;
                    target.active = false;
                    adv_speed = MAX_ADV_SPEED *dist *s;
                    qInfo()<<"case 0";
                    break;
                case 1:
                    //ir hacia el punto
                    dist = 1;
                    adv_speed = MAX_ADV_SPEED *dist *s;
                    qInfo()<<"case 1";
                    break;
                case 2:
                    //obstaculo cercano hay que girar
                    girar();
                    qInfo()<<"case 2";
                    break;
            }
            //Siempre se dirige al objetivo
            differentialrobot_proxy->setSpeedBase(adv_speed,rot_speed);
        } catch (const Ice::Exception &e) {
            std::cout << e.what() << std::endl;
        }
    }
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState bState) // robot coordinates
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

std::tuple<float, float> SpecificWorker::world2robot(RoboCompGenericBase::TBaseState bState)
{
    float angle = bState.alpha;
    Eigen::Vector2f T(bState.x, bState.z), point_in_world(target.pos.x(),target.pos.y());
    Eigen::Matrix2f R;
    R << cos(angle), -sin(angle), sin(angle), cos(angle);
    Eigen::Vector2f point_in_robot = R.transpose() * (point_in_world - T);
    return make_tuple(point_in_robot[0],point_in_robot[1]);
}



void SpecificWorker::girar()
{
    int j=0;
    while(j<1000) {
        this->differentialrobot_proxy->setSpeedBase(10, 1);
        j++;
    }
    this->differentialrobot_proxy->setSpeedBase(0,0);
    this->differentialrobot_proxy->stopBase();
    j=0;
    while(j<1000) {
        this->differentialrobot_proxy->setSpeedBase(100, 0);
        j++;
    }
    this->differentialrobot_proxy->setSpeedBase(0,0);
    this->differentialrobot_proxy->stopBase();
    j=0;
}



