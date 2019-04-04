#include "artifical_potential.h"
#include "utils/datamanager.h"
#include "utils/params.h"
#include "communication/udpsender.h"
#include <cmath>
#include <iostream>
#include <iomanip>

namespace{
const int m = 3;
const int n = 3;
const float alpha_p = 1.0;
const float alpha_v = 0;
const float m_attractive = 1000;  // virtual mass of attractive potential field
const float m_repulsive = 1.0;  // virtual mass of repulsive potential field
const float MAX_ACC = 1.2;
const float MAX_SPEED = 0.4;
const float eta = 150.0;  // positive constant
const float rou_0 = 50;  // a positive constant describing the influence range of
                        // the obstacle
}

//void ArtificalPotential::plan( MyPoint target, MyVector v_target ){
bool ArtificalPotential::plan( MyPoint target ){
    bool if_use_artifical_potential = false;
    std::cout << "\n==============  [artifical_potenial.cpp] debug  =========="
                    "=====" << std::endl;
//    RobotInfo& me = MyDataManager::instance()->blueRobots[1];
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    MyVector v_target( 0, 0 ); //////////////////////////////////////
    MyPoint me_pos( me.x, me.y );
//    MyVector me_vel( me.vel_x, -me.vel_y );
        MyVector me_vel( me.vel_x, me.vel_y );
    MyVector f_attractive =
            ( target - me_pos ) * m * alpha_p *
            pow( (target - me_pos).mod(), m - 2 )
             + ( v_target - me_vel ) * n * alpha_v *
            pow( ( v_target - me_vel ).mod(), n - 2 );
    MyVector acc_attractive = f_attractive / m_attractive ; // simulation
    acc_attractive.Sety(-acc_attractive.y());

    float rou_s[PARAMS::ROBOT_NUM];
    float rou_m[PARAMS::ROBOT_NUM];
    MyVector f_rep1[PARAMS::ROBOT_NUM];
    MyVector f_rep2[PARAMS::ROBOT_NUM];
    MyVector f_rep( 0, 0 );
    for ( int i = 0; i < PARAMS::ROBOT_NUM; i++ ){
        if ( MyDataManager::instance()->validYellowRobots[i] ){
            MyPoint yellow_pos( MyDataManager::instance()->yellowRobots[i].x,
                                MyDataManager::instance()->yellowRobots[i].y );
            std::cout << "Yellow id: " << i << std::endl;
//            std::cout << "[ap.cpp] \nyellow robots x: " << std::right << std::setw(12) << yellow_pos.x()
//                      << ", yellow robots y: " << std::right << std::setw(12) << yellow_pos.y()
//                      << "\n-------------------------------------------------------"
//                         "-------" << std::endl;
            MyVector yellow_vel(
                        MyDataManager::instance()->yellowRobots[i].vel_x,
                        MyDataManager::instance()->yellowRobots[i].vel_y);
            rou_s[i] = ( me_pos - yellow_pos ).mod();
            MyVector n_ro = ( yellow_pos - me_pos ).Unitization();
            MyVector v_ro = n_ro * ( ( me_vel - yellow_vel ) * n_ro );
            MyVector v_ro_vertical = me_vel - yellow_vel - v_ro;
            MyVector n_ro_vertical = v_ro_vertical.Unitization();
            rou_m[i] = v_ro.mod() * v_ro.mod() / 2 / 1000; ///////////////////
            std::cout << "rou_s[i] - rou_m[i]           : "
                      << std::right << std::setw(12)
                      << rou_s[i] - rou_m[i]
                      << std::endl;
            std::cout << "( me_vel - yellow_vel ) * n_ro: "
                      << std::right << std::setw(12)
                      << ( me_vel - yellow_vel ) * n_ro
                      << std::endl;
            std::cout << "(me_vel - yellow_vel) x       : "
                      << std::right << std::setw(12)
                      << (me_vel - yellow_vel).x()
                      << "\n(me_vel - yellow_vel) y       : "
                      << std::right << std::setw(12)
                      << (me_vel - yellow_vel).y()
                      << std::endl;
            std::cout << "n_ro x                        : "
                      << std::right << std::setw(12) << n_ro.x()
                      << "\nn_ro y                        : "
                      << std::right << std::setw(12) << n_ro.y()
                      << std::endl
                      << std::endl;
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
                f_rep = f_rep + f_rep1[i] + f_rep2[i];
                if_use_artifical_potential = true;
                std::cout << "[ap.cpp] in else" << std::endl;
            }
            else{
                std::cout << "[ap.cpp] ERROR, crush happen!" << std::endl;
            }
        }
    }

    MyVector acc_repulsive = f_rep / m_repulsive;  // simulation
    acc_repulsive.Sety(-acc_repulsive.y());
    MyVector acc_tol = acc_attractive + acc_repulsive;
    if ( acc_tol.mod() > MAX_ACC ){
        acc_tol = acc_tol.Unitization() * MAX_ACC;
    }

    MyVector next_step_vel = me_vel / 50 + acc_tol;
    if ( next_step_vel.mod() > MAX_SPEED ){
        next_step_vel = next_step_vel.Unitization() * MAX_SPEED;
    }

    float dir = me.orientation;
//    v_x = next_step_vel.x() * cos(dir) + next_step_vel.y() * sin(dir);
//    v_y = next_step_vel.x() * sin(dir) - next_step_vel.y() * cos(dir);
    v_x = next_step_vel.x() * cos(dir) + next_step_vel.y() * sin(dir);
    v_y = next_step_vel.x() * sin(dir) - next_step_vel.y() * cos(dir);
//    v_x = 0.5;
//    v_y = 0.5;
    std::cout << "[ap.cpp] \nacc_attractive_x: " << std::right << std::setw(12) << acc_attractive.x()
              << ", acc_attractive_y: " << std::right << std::setw(12) << acc_attractive.y()
              << "\nacc_repulsive_x : " << std::right << std::setw(12) << acc_repulsive.x()
              << ", acc_repulsive_y : " << std::right << std::setw(12) << acc_repulsive.y()
              << "\nacc_tol_x       : " << std::right << std::setw(12) << acc_tol.x()
              << ", acc_tol_y       : " << std::right << std::setw(12) <<  acc_tol.y()
              << "\nme_vel_x        : " << std::right << std::setw(12) << me.vel_x
              << ", me_vel_y        : " << std::right << std::setw(12) << me.vel_y
              << "\nnext_step_vel_x : " << std::right << std::setw(12) << next_step_vel.x()
              << ", next_step_vel_y : " << std::right << std::setw(12) << next_step_vel.y()
              << "\nv_x             : " << std::right << std::setw(12) << v_x
              << ", v_y             : " << std::right << std::setw(12) << v_y
              << "\ndir             : " << std::right << std::setw(12) << dir
              << std::endl;
//    v_x = 1;
//    v_y = 1;
    std::cout << "\n=========================================================="
                    "=====" << std::endl;
    return if_use_artifical_potential;
}
