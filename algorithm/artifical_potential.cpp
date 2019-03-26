#include "artifical_potential.h"
#include "utils/datamanager.h"
#include "utils/params.h"
#include "communication/udpsender.h"
#include <cmath>
#include <iostream>

namespace{
const int m = 6;
const int n = 6;
const float alpha_p = 10000.0;
const float alpha_v = 1.0;
const float m_attractive = 1000000000;  // virtual mass of attractive potential field
const float m_repulsive = 0.000000001;  // virtual mass of repulsive potential field
const float MAX_ACC = 8000;
const float MAX_SPEED = 0.4;
const float eta = 10000000000.0;  // positive constant
const float rou_0 = 150;  // a positive constant describing the influence range of
                        // the obstacle
}

//void ArtificalPotential::plan( MyPoint target, MyVector v_target ){
void ArtificalPotential::plan( MyPoint target ){
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    MyVector v_target( 0, 0 ); //////////////////////////////////////
    MyPoint me_pos( me.x, me.y );
    MyVector me_vel( me.vel_x, -me.vel_y );
    MyVector f_attractive =
            ( target - me_pos ) * m * alpha_p *
            pow( (target - me_pos).mod(), m - 2 )
             + ( v_target - me_vel ) * n * alpha_v *
            pow( ( v_target - me_vel ).mod(), n - 2 );
    MyVector acc_attractive = f_attractive / m_attractive ;

    float rou_s[PARAMS::ROBOT_NUM];
    float rou_m[PARAMS::ROBOT_NUM];
    MyVector f_rep1[PARAMS::ROBOT_NUM];
    MyVector f_rep2[PARAMS::ROBOT_NUM];
    MyVector f_rep( 0, 0 );
    for ( int i = 0; i < PARAMS::ROBOT_NUM; i++ ){
        if ( MyDataManager::instance()->validYellowRobots[i] ){
            MyPoint yellow_pos( MyDataManager::instance()->yellowRobots[i].x,
                                MyDataManager::instance()->yellowRobots[i].y );
            MyVector yellow_vel(
                        MyDataManager::instance()->yellowRobots[i].vel_x,
                        MyDataManager::instance()->yellowRobots[i].vel_y);
            rou_s[i] = ( me_pos - yellow_pos ).mod();
            MyVector n_ro = ( yellow_pos - me_pos ).Unitization();
            MyVector v_ro = n_ro * ( ( me_vel - yellow_vel ) * n_ro );
            MyVector v_ro_vertical = me_vel - yellow_vel - v_ro;
            MyVector n_ro_vertical = v_ro_vertical.Unitization();
            rou_m[i] = v_ro.mod() * v_ro.mod() / 2 / 1000; ///////////////////
//            std::cout << "[ap.cpp] " << rou_m[i] << ", " << rou_s[i] << std::endl;
            if ( rou_s[i] - rou_m[i] >= rou_0 ||
                 ( me_vel - yellow_vel ) * n_ro <= 0 ){
            }
            else if ( rou_s[i] - rou_m[i] < rou_0 && rou_s[i] - rou_m[i] > 0 &&
                      ( me_vel - yellow_vel ) * n_ro > 0 ){
                f_rep1[i] = n_ro * (-1) * eta * ( 1 + v_ro.mod() / MAX_ACC ) /
                        ( rou_s[i] - rou_m[i] ) / ( rou_s[i] - rou_m[i] );
                f_rep2[i] = n_ro_vertical *
                        eta * v_ro.mod() * v_ro_vertical.mod() / rou_s[i]
                        / MAX_ACC / ( rou_s[i] - rou_m[i] ) /
                        ( rou_s[i] - rou_m[i] );
//                std::cout << "[ap.cpp] " << f_rep1[i].y() << ", " << f_rep2[i].y() << ", " << rou_m[i] << ", " << rou_s[i] << std::endl;
                f_rep = f_rep + f_rep1[i] + f_rep2[i];
            }
            else{
                std::cout << "ERROR, crush happen!" << std::endl;
            }
        }
    }

    MyVector acc_repulsive = f_rep / m_repulsive;
    MyVector acc_tol = acc_attractive + acc_repulsive;
//    std::cout << "[ap.cpp]x: " << acc_tol.x() << ", " << acc_tol.y() << std::endl;
//    MyVector acc_tol = acc_repulsive;
    if ( acc_tol.mod() > MAX_ACC ){
//        std::cout << "in if" << std::endl;
        acc_tol = acc_tol.Unitization() * MAX_ACC;
    }
    MyVector next_step_vel = me_vel + acc_tol / PARAMS::FRAME;
    std::cout << "[ap.cpp]x: " << me_vel.mod() << ", " << acc_tol.y() << ", " << next_step_vel.mod() << std::endl;

//    std::cout << acc_tol.y() << ", " << next_step_vel.y() << std::endl;
    if ( next_step_vel.mod() > MAX_SPEED ){
//        std::cout << "in if2" << std::endl;
//        std::cout << next_step_vel.x() << ", " << next_step_vel.y() << ", " << next_step_vel.Unitization().x() << ", " << next_step_vel.Unitization().y()
//                  << "; " <<  (next_step_vel.Unitization() * MAX_SPEED).x() << ", " << (next_step_vel.Unitization() * MAX_SPEED).y();
        next_step_vel = next_step_vel.Unitization() * MAX_SPEED;
//        std::cout << ", " <<next_step_vel.x() << ", " << next_step_vel.y() << std::endl;
    }
//    std::cout << "[ap.cpp] x: " << next_step_vel.x() << ", " << next_step_vel.y() << std::endl;
    float dir = me.orientation;
    float v_x = next_step_vel.x() * cos(dir) + next_step_vel.y() * sin(dir);
    float v_y = next_step_vel.x() * sin(dir) - next_step_vel.y() * cos(dir);
//    std::cout << next_step_vel.x() << ", " << next_step_vel.y() << ", " << sin(dir) << ", " << cos(dir) << ", " << v_x << ", " << v_y << std::endl;
//    std::cout << me.vel_x << "," << me.vel_y << std::endl;
    CommandSender::instance()->sendToSim( me.robot_id, v_x, v_y, 0);
}
