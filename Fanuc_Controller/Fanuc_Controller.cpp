#include "webots/motor.h"
#include "webots/camera.h"
#include "webots/robot.h"
#include "webots/distance_sensor.h"
#include "webots/range_finder.h"
#include <webots/position_sensor.h>

#include "ControlRobot.h"
#include "track_gen.h"

#include <iostream>
#include <chrono>
#include <thread>

#define TIME_STEP 16
#define MAX_SPEED 6.28


int main(int argc, char **argv)
{
    wb_robot_init();

//    WbDeviceTag Camera = wb_robot_get_device("Terahertz");
//    wb_camera_enable(Camera, TIME_STEP);

//    WbDeviceTag DistanceLaser = wb_robot_get_device("DistanceLaser");
//    wb_distance_sensor_enable(DistanceLaser, TIME_STEP);

//    WbDeviceTag RealSense = wb_robot_get_device("RealSense");
//    wb_range_finder_enable(RealSense, TIME_STEP);

    // get the motor devices Fanuc
    WbDeviceTag Joint_1 = wb_robot_get_device("Joint_1");
    WbDeviceTag Joint_2 = wb_robot_get_device("Joint_2");
    WbDeviceTag Joint_3 = wb_robot_get_device("Joint_3");
    WbDeviceTag Joint_4 = wb_robot_get_device("Joint_4");
    WbDeviceTag Joint_5 = wb_robot_get_device("Joint_5");
    WbDeviceTag Joint_6 = wb_robot_get_device("Joint_6");

    // set the target position of the motors
    wb_motor_set_position(Joint_1, INFINITY);
    wb_motor_set_position(Joint_2, INFINITY);
    wb_motor_set_position(Joint_3, INFINITY);
    wb_motor_set_position(Joint_4, INFINITY);
    wb_motor_set_position(Joint_5, INFINITY);
    wb_motor_set_position(Joint_6, INFINITY);


    WbDeviceTag Joint_1_PositionSensor = wb_motor_get_position_sensor(Joint_1);
    WbDeviceTag Joint_2_PositionSensor = wb_motor_get_position_sensor(Joint_2);
    WbDeviceTag Joint_3_PositionSensor = wb_motor_get_position_sensor(Joint_3);
    WbDeviceTag Joint_4_PositionSensor = wb_motor_get_position_sensor(Joint_4);
    WbDeviceTag Joint_5_PositionSensor = wb_motor_get_position_sensor(Joint_5);
    WbDeviceTag Joint_6_PositionSensor = wb_motor_get_position_sensor(Joint_6);

    wb_position_sensor_enable(Joint_1_PositionSensor, TIME_STEP);
    wb_position_sensor_enable(Joint_2_PositionSensor, TIME_STEP);
    wb_position_sensor_enable(Joint_3_PositionSensor, TIME_STEP);
    wb_position_sensor_enable(Joint_4_PositionSensor, TIME_STEP);
    wb_position_sensor_enable(Joint_5_PositionSensor, TIME_STEP);
    wb_position_sensor_enable(Joint_6_PositionSensor, TIME_STEP);

    ControlRobot Robot;

    /////////////////
    Eigen::Affine3f final_pose;
    final_pose.translation().matrix()<<1,2,3;

    //////

    while (wb_robot_step(TIME_STEP) != -1)
    {
        //float Joint1Position = wb_position_sensor_get_value(Joint_1_PositionSensor);
        //float Joint2Position = wb_position_sensor_get_value(Joint_2_PositionSensor);
        //float Joint3Position = wb_position_sensor_get_value(Joint_3_PositionSensor);
        //float Joint4Position = wb_position_sensor_get_value(Joint_4_PositionSensor);
        //float Joint5Position = wb_position_sensor_get_value(Joint_5_PositionSensor);
        //float Joint6Position = wb_position_sensor_get_value(Joint_6_PositionSensor);
//
        ////Robot.MatriceTransformationDHFanuc(Joint1Position, Joint2Position, Joint3Position, Joint4Position, Joint5Position, Joint6Position);
        ////Robot.MatriceJacobienne();
        ////Robot.MatriceJacobienneInverse();
        ////Eigen::Matrix<float, 6, 1> ResultatKinematic = Robot.InverseKinematic();

        wb_motor_set_velocity(Joint_1, .0);
        wb_motor_set_velocity(Joint_2, .0);
        wb_motor_set_velocity(Joint_3, .0);
        wb_motor_set_velocity(Joint_4, .0);
        wb_motor_set_velocity(Joint_5, .0);
        wb_motor_set_velocity(Joint_6, .0);


       // Track track1(10,.01);

      //  std::cout<<track1;
        RuckigTrajectory track1(final_pose, 20,.01, Robot);
        std::cout<<track1.best_track.pose[0].matrix()<<std::endl;
//std::cout<<track1.steps<<std::endl;
       // std::cout<<track1.pose(0)<<std::endl;

    break;
    };

    wb_robot_cleanup();

    return 0;
}
