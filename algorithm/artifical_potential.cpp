#include "artifical_potential.h"
#include "utils/datamanager.h"
#include "utils/params.h"
#include "communication/udpsender.h"
#include <cmath>
#include <iostream>

namespace{
const int m = 2;
const int n = 2;
const float alpha_p = 1.0;
const float alpha_v = 1.0;
const float m_attractive = 1.0;  // virtual mass of attractive potential field
const float m_repulsive = 1.0;  // virtual mass of repulsive potential field
const float MAX_ACC = 600;
const float eta = 1.0;  // positive constant
const float rou_0 = 0;  // a positive constant describing the influence range of
                        // the obstacle
}

void ArtificalPotential::plan( MyPoint target, MyVector v_target ){
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    MyPoint me_pos( me.x, me.y );
    MyVector me_vel( me.vel_x, me.vel_y );
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
            rou_m[i] = v_ro.mod() * v_ro.mod() / 2 / MAX_ACC;
            if ( rou_s[i] - rou_m[i] >= rou_0 ||
                 ( me_vel - yellow_vel ) * n_ro <= 0 );
            else if ( rou_s[i] - rou_m[i] < rou_0 && rou_s[i] - rou_m[i] > 0 &&
                      ( me_vel - yellow_vel ) * n_ro > 0 ){
                f_rep1[i] = n_ro * (-1) * eta * ( 1 + v_ro.mod() / MAX_ACC ) /
                        ( rou_s[i] - rou_m[i] ) / ( rou_s[i] - rou_m[i] );
                f_rep2[i] = n_ro_vertical *
                        eta * v_ro.mod() * v_ro_vertical.mod() / rou_s[i]
                        / MAX_ACC / ( rou_s[i] - rou_m[i] ) /
                        ( rou_s[i] - rou_m[i] );
                f_rep = f_rep + f_rep1[i] + f_rep2[i];
            }
            else{
                std::cout << "ERROR, crush happen!" << std::endl;
            }
        }
    }
    MyVector acc_repulsive = f_rep / m_repulsive;
    MyVector acc_tol = acc_attractive + acc_repulsive;
    MyVector next_step_vel = me_vel + acc_tol * PARAMS::FRAME;
    CommandSender::instance()->sendToSim( me.robot_id, next_step_vel.x(), next_step_vel.y());
}
