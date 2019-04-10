#include "artifical_potential.h"
#include "utils/datamanager.h"
#include "utils/params.h"
#include "communication/udpsender.h"
#include <cmath>
#include <iostream>
#include <iomanip>

// Athena     output unit mm
// simulation output unit m
// serial     output unit cm

namespace{
const int m = 3;
const int n = 2;
const float alpha_p = 1.0;
const float alpha_v = 5;
// virtual mass of attractive potential field
// for sim
//const float m_attractive = 1200;
// for real
const float m_attractive = 12;
// virtual mass of repulsive potential field
// for sim
//const float m_repulsive = 0.05;
// for real
const float m_repulsive = 0.00005;
//const float MAX_ACC = 1000;
//const float MAX_SPEED = 1500;
const float MAX_ACC = 3500;
const float MAX_SPEED = 5500;
const float ACC_BUFFER = 2;
const float RATE = 0.2;
const float REP_RATE = 0.8;
//const float eta = 7500.0;  // positive constant
//const float rou_0 = 110;  // a positive constant describing the influence range of
//                        // the obstacle
const float eta = 11000.0;  // positive constant
const float rou_0 = 110;  // a positive constant describing the influence range of
                        // the obstacle
const float DELTA_TIME  = 0.03;  // unit is second
}

// change Athena axis y up
//void ArtificalPotential::plan( MyPoint target, MyVector v_target ){
bool ArtificalPotential::plan( MyPoint target ){
    target.Sety(-target.y());
    bool if_use_artifical_potential = false;
    if ( PARAMS::DEBUG::kAPDebugMessage ){
        std::cout << "\n==============  [artifical_potenial.cpp] debug  ======="
                        "========" << std::endl;
    }

    RobotInfo& me = MyDataManager::instance()->ourRobot();
    MyVector v_target( 0, 0 );
    MyPoint me_pos( me.x, -me.y );
    MyVector me_vel;
    me_vel.Setx(me.vel_x);
    me_vel.Sety(me.vel_y);

    if ( PARAMS::DEBUG::kAPDebugMessage ) {
//        std::cout << "me_vel.x                      : "
//                  << std::right << std::setw(12)
//                  << me_vel.x()
//                  << ", \nme_vel.y                      : "
//                  << std::right << std::setw(12)
//                  << me_vel.y()
//                  << std::endl;
//        std::cout << "me.vel_x                      : "
//                  << std::right << std::setw(12)
//                  << me.vel_x
//                  << ", \nme.vel_y                      : "
//                  << std::right << std::setw(12)
//                  << me.vel_y
//                  << std::endl;
    }

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
                                -MyDataManager::instance()->yellowRobots[i].y );

            if ( PARAMS::DEBUG::kAPDebugMessage ) {
                std::cout << "Yellow id: " << i << std::endl;
            }

            MyVector yellow_vel(
                        MyDataManager::instance()->yellowRobots[i].vel_x,
                        MyDataManager::instance()->yellowRobots[i].vel_y);
            rou_s[i] = ( me_pos - yellow_pos ).mod();
            MyVector n_ro = ( yellow_pos - me_pos ).Unitization();
            MyVector v_ro = n_ro * ( ( me_vel - yellow_vel ) * n_ro );
            MyVector v_ro_vertical = me_vel - yellow_vel - v_ro;
            MyVector n_ro_vertical = v_ro_vertical.Unitization();
            rou_m[i] = v_ro.mod() * v_ro.mod() / 2 / MAX_ACC;

            bool if_rep;
            if ( !last_statue )
                if_rep = rou_s[i] - rou_m[i] < rou_0 &&
                    rou_s[i] - rou_m[i] > 0 &&
                    ( me_vel - yellow_vel ) * n_ro > 0;
            else
                if_rep = rou_s[i] - rou_m[i] < rou_0 * REP_RATE &&
                        rou_s[i] - rou_m[i] > 0 &&
                        ( me_vel - yellow_vel ) * n_ro > 0;
            last_statue = false;

            if ( PARAMS::DEBUG::kAPDebugMessage ) {
//                std::cout << "rou_s[i] - rou_m[i]           : "
//                          << std::right << std::setw(12)
//                          << rou_s[i] - rou_m[i]
//                          << "\n"
//                          << std::endl;
//                std::cout << "( me_vel - yellow_vel ) * n_ro: "
//                          << std::right << std::setw(12)
//                          << ( me_vel - yellow_vel ) * n_ro
//                          << std::endl;
//                std::cout << "(me_vel - yellow_vel) x       : "
//                          << std::right << std::setw(12)
//                          << (me_vel - yellow_vel).x()
//                          << "\n(me_vel - yellow_vel) y       : "
//                          << std::right << std::setw(12)
//                          << (me_vel - yellow_vel).y()
//                          << std::endl;
//                std::cout << "n_ro x                        : "
//                          << std::right << std::setw(12) << n_ro.x()
//                          << "\nn_ro y                        : "
//                          << std::right << std::setw(12) << n_ro.y()
//                          << std::endl
//                          << std::endl;
            }

            if ( rou_s[i] - rou_m[i] >= rou_0 ||
                 ( me_vel - yellow_vel ) * n_ro <= 0 ){

            }
            else if ( if_rep ){
                f_rep1[i] = n_ro * (-1) * eta * ( 1 + v_ro.mod() / MAX_ACC ) /
                        ( rou_s[i] - rou_m[i] ) / ( rou_s[i] - rou_m[i] );
                f_rep2[i] = n_ro_vertical *
                        eta * v_ro.mod() * v_ro_vertical.mod() / rou_s[i]
                        / MAX_ACC / ( rou_s[i] - rou_m[i] ) /
                        ( rou_s[i] - rou_m[i] );
                f_rep = f_rep + f_rep1[i] + f_rep2[i];
                if ( yellow_vel.mod() > 300 ){
                    if_use_artifical_potential = true;
                    if (PARAMS::DEBUG::kAPDebugMessage){
                        std::cout << "[artifical_potential.cpp]"
                                  << "\nyellow_vel.mod: " << yellow_vel.mod()
                                  << std::endl;
                    }
                }
            }
            else{
                if ( PARAMS::DEBUG::kAPDebugMessage ){
                    std::cout << "[artifical_potential.cpp] not defined!\n"
                              << std::endl;
                }
            }
        }
    }
    for ( int i = 0; i < PARAMS::ROBOT_NUM; i++ ){
        if ( MyDataManager::instance()->validBlueRobots[i] &&
             i != PARAMS::our_id - 1 ){
            MyPoint blue_pos( MyDataManager::instance()->blueRobots[i].x,
                               -MyDataManager::instance()->blueRobots[i].y );

            if ( PARAMS::DEBUG::kAPDebugMessage ) {
                std::cout << "Blue id: " << i << std::endl;
            }

            MyVector blue_vel(
                        MyDataManager::instance()->blueRobots[i].vel_x,
                        MyDataManager::instance()->blueRobots[i].vel_y);
            rou_s[i] = ( me_pos - blue_pos ).mod();
            MyVector n_ro = ( blue_pos - me_pos ).Unitization();
            MyVector v_ro = n_ro * ( ( me_vel - blue_vel ) * n_ro );
            MyVector v_ro_vertical = me_vel - blue_vel - v_ro;
            MyVector n_ro_vertical = v_ro_vertical.Unitization();
            rou_m[i] = v_ro.mod() * v_ro.mod() / 2 / MAX_ACC;

            bool if_rep;
            if ( !last_statue )
                if_rep = rou_s[i] - rou_m[i] < rou_0 &&
                    rou_s[i] - rou_m[i] > 0 &&
                    ( me_vel - blue_vel ) * n_ro > 0;
            else
                if_rep = rou_s[i] - rou_m[i] < rou_0 * REP_RATE &&
                        rou_s[i] - rou_m[i] > 0 &&
                        ( me_vel - blue_vel ) * n_ro > 0;
            last_statue = false;
            if ( PARAMS::DEBUG::kAPDebugMessage ) {
//                std::cout << "rou_s[i] - rou_m[i]           : "
//                          << std::right << std::setw(12)
//                          << rou_s[i] - rou_m[i]
//                          << "\n"
//                          << std::endl;
//                std::cout << "( me_vel - blue_vel ) * n_ro  : "
//                          << std::right << std::setw(12)
//                          << ( me_vel - blue_vel ) * n_ro
//                          << std::endl;
//                std::cout << "(me_vel - blue_vel) x         : "
//                          << std::right << std::setw(12)
//                          << (me_vel - blue_vel).x()
//                          << "\n(me_vel - blue_vel) y         : "
//                          << std::right << std::setw(12)
//                          << (me_vel - blue_vel).y()
//                          << std::endl;
//                std::cout << "n_ro x                        : "
//                          << std::right << std::setw(12) << n_ro.x()
//                          << "\nn_ro y                        : "
//                          << std::right << std::setw(12) << n_ro.y()
//                          << std::endl
//                          << std::endl;
            }

            if ( rou_s[i] - rou_m[i] >= rou_0 ||
                 ( me_vel - blue_vel ) * n_ro <= 0 ){
            }
            else if ( if_rep ){
                f_rep1[i] = n_ro * (-1) * eta * ( 1 + v_ro.mod() / MAX_ACC ) /
                        ( rou_s[i] - rou_m[i] ) / ( rou_s[i] - rou_m[i] );
                f_rep2[i] = n_ro_vertical *
                        eta * v_ro.mod() * v_ro_vertical.mod() / rou_s[i]
                        / MAX_ACC / ( rou_s[i] - rou_m[i] ) /
                        ( rou_s[i] - rou_m[i] );
                f_rep = f_rep + f_rep1[i] + f_rep2[i];
                if ( blue_vel.mod() > 300 ){
                    if_use_artifical_potential = true;
                    if (PARAMS::DEBUG::kAPDebugMessage){
                        std::cout << "[artifical_potential.cpp]"
                                  << "\nblue_vel.mod: " << blue_vel.mod()
                                  << std::endl;
                    }
                }
            }
            else{
                if ( PARAMS::DEBUG::kAPDebugMessage ){
                    std::cout << "[artifical_potential.cpp] not defined!\n"
                              << std::endl;
                }
            }
        }
    }

    MyVector acc_repulsive = f_rep / m_repulsive;

//    MyVector acc_tol = acc_attractive + acc_repulsive;
    MyVector acc_tol;
    if ( me_vel.mod() > MAX_SPEED * RATE ){
        acc_tol = acc_repulsive;
    }
    else{
        acc_tol = acc_attractive + acc_repulsive;
    }

    if ( acc_tol.mod() > MAX_ACC ){
        acc_tol = acc_tol.Unitization() * MAX_ACC;
    }

    MyVector next_step_vel;
    next_step_vel = me_vel + acc_tol * DELTA_TIME * ACC_BUFFER;

    if ( next_step_vel.mod() > MAX_SPEED ){
        next_step_vel = next_step_vel.Unitization() * MAX_SPEED;
    }

    float dir = me.orientation;
    if ( PARAMS::IS_SIMULATION ){
        v_x = next_step_vel.x() * cos(dir) + next_step_vel.y() * sin(dir);
        v_y = - next_step_vel.x() * sin(dir) + next_step_vel.y() * cos(dir);
    }
    else {
        v_x = next_step_vel.x() * cos(dir) + next_step_vel.y() * sin(dir);
        v_y = next_step_vel.x() * sin(dir) - next_step_vel.y() * cos(dir);
    }

    if ( PARAMS::DEBUG::kAPDebugMessage ) {
//        std::cout << "[ap.cpp] \nacc_attractive_x: " << std::right << std::setw(12) << acc_attractive.x()
//                  << ", acc_attractive_y: " << std::right << std::setw(12) << acc_attractive.y()
//                  << "\nacc_repulsive_x : " << std::right << std::setw(12) << acc_repulsive.x()
//                  << ", acc_repulsive_y : " << std::right << std::setw(12) << acc_repulsive.y()
//                  << "\nacc_tol_x       : " << std::right << std::setw(12) << acc_tol.x()
//                  << ", acc_tol_y       : " << std::right << std::setw(12) <<  acc_tol.y()
//                  << "\nme_vel_x        : " << std::right << std::setw(12) << me.vel_x
//                  << ", me_vel_y        : " << std::right << std::setw(12) << me.vel_y
//                  << "\nnext_step_vel_x : " << std::right << std::setw(12) << next_step_vel.x()
//                  << ", next_step_vel_y : " << std::right << std::setw(12) << next_step_vel.y()
//                  << "\nv_x             : " << std::right << std::setw(12) << v_x
//                  << ", v_y             : " << std::right << std::setw(12) << v_y
//                  << "\ndir             : " << std::right << std::setw(12) << dir
//                  << std::endl;
        std::cout << "\n=========================================================="
                        "=====" << std::endl;
    }

    return if_use_artifical_potential;
}
