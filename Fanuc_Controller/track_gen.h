#include <iostream>
#include <cmath>
#include <ruckig/ruckig.hpp>
#include "ControlRobot.h"
#include <vector>

using namespace ruckig;

class Track
{
public:
    Track();
    Track(double time, double res);
    std::vector<Eigen::Affine3f> pose;
    double steps;
};

class RuckigTrajectory
{
public:
    Eigen::Affine3f final_pose;
    Eigen::Affine3f actual_pose;
    double time;
    double resolution;
    Track best_track;


    //RuckigTrajectory();
    RuckigTrajectory(Eigen::Affine3f, double,double, ControlRobot); // implemented
    //~RuckigTrajectory();

   void generate(Track&);

   bool check_for_sing(Track&);
   bool check_for_DZ(Track&);
   bool check_kinematic_limits(Track&);
   void rot_interpolate(Track&);
   //void recalculate(Track&);

};






//class RuckigTrajectory
//{
//public:
//    // RuckigTrajectory();
//    RuckigTrajectory(Eigen::Affine3f, double, ControlRobot);
//    // ~RuckigTrajectory();
//
//    Track generate();
//    bool check_for_sing();
//    bool check_for_DZ();
//    bool check_kinematic_limits();
//    void rot_interpolate(Track&);
//    Track recalculate();
//
//    Track best_track;
//    Eigen::Affine3f final_pose;
//    Eigen::Affine3f actual_pose;
//    double time;
//    double resolution;
//};
//























//t main() {
//  // Create instances: the Ruckig OTG as well as input and output parameters
//  Ruckig<3> otg {0.01};  // control cycle
//  InputParameter<3> input;
//  OutputParameter<3> output;
//
//  // Set input parameters
//  input.current_position = {0.0, 0.0, 0.5};
//  input.current_velocity = {0.0, -2.2, -0.5};
//  input.current_acceleration = {0.0, 2.5, -0.5};
//
//  input.target_position = {5.0, -2.0, -3.5};
//  input.target_velocity = {0.0, -0.5, -2.0};
//  input.target_acceleration = {0.0, 0.0, 0.5};
//
//  input.max_velocity = {3.0, 1.0, 3.0};
//  input.max_acceleration = {3.0, 2.0, 1.0};
//  input.max_jerk = {4.0, 3.0, 2.0};
//
//  // Generate the trajectory within the control loop
//  std::cout << "t | p1 | p2 | p3" << std::endl;
//  while (otg.update(input, output) == Result::Working) {
//      auto& p = output.new_position;
//      std::cout << output.time << " " << p[0] << " " << p[1] << " " << p[2] << " " << std::endl;
//
//      output.pass_to_input(input);
//  }
//
//  std::cout << "Trajectory duration: " << output.trajectory.get_duration() << " [s]." << std::endl;
//
//
