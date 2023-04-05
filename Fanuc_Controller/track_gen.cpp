#include "track_gen.h"

Track::Track(){
};

Track::Track(double time, double resol)
{
    this->steps=time/resol;
    // Create Identity Matrices
    for (int i = 0; i <steps; i++)
    {
        this->pose.push_back(Eigen::Affine3f::Identity());
    };
};

RuckigTrajectory::RuckigTrajectory(Eigen::Affine3f desired_pose, double final_time,double resol, ControlRobot Robot)
{
    this->time = final_time;
    this->final_pose = desired_pose;
    this->resolution = resol ;
    this->actual_pose.matrix() = Robot.TransformationMatrice;

    Track tmp_track(this->time,this->resolution);

    this->best_track =tmp_track;
    generate(this->best_track);
};

void RuckigTrajectory::generate(Track& tr)
{
    //Track tr(this->time, this->resolution);

    ruckig::InputParameter<3> input;

    input.current_position = {this->actual_pose.matrix()(0, 3), this->actual_pose.matrix()(1, 3), this->actual_pose.matrix()(2, 3)};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};

    input.target_position = {this->final_pose.matrix()(0, 3), this->final_pose.matrix()(1, 3), this->final_pose.matrix()(2, 3)};
    input.target_velocity = {0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};

    input.max_velocity = {3.0, 1.0, 3.0};
    input.max_acceleration = {3.0, 2.0, 1.0};
    input.max_jerk = {4.0, 3.0, 2.0};

    ruckig::Ruckig<3> otg;
    ruckig::Trajectory<3> trajectory_ruckig;

    std::cout << "high from the class declaration!" << std::endl;
    // Calculate the trajectory in an offline manner (outside of the control loop)
    ruckig::Result result = otg.calculate(input, trajectory_ruckig);
    if (result == ruckig::Result::ErrorInvalidInput)
    {
        std::cout << "Invalid input!" << std::endl;
    }

    std::array<double, 3> new_position, new_velocity, new_acceleration;
    Eigen::Vector3f tmp_pose;
    int i = 0;

    // std::cout<<"\t"<<"t"<<"\t\t"<<"i"<<std::endl;

        float res2;
        res2= trajectory_ruckig.get_duration()*this->resolution/this->time;
    for (float t = 0; t < trajectory_ruckig.get_duration(); t += res2)
    {

        trajectory_ruckig.at_time(t, new_position, new_velocity, new_acceleration);
        tmp_pose << new_position[0], new_position[1], new_position[2];
        tr.pose[i].translation().matrix() = tmp_pose;
        i++;
    }


    //// Get some info about the position extrema of the trajectory
    //// std::array<PositionExtrema, 3> position_extrema = trajectory.get_position_extrema();
    //// std::cout << "Position extremas for DoF 4 are " << position_extrema[2].min << " (min) to " << position_extrema[2].max << " (max)" << std::endl;

}

bool RuckigTrajectory::check_for_sing(Track& tr)
{
    bool state = true;
    //! model each link as an elipsoid
    //! check for defined danger zones
    //! go through all the jacobians and calculate the determinants
    return state;
};

bool RuckigTrajectory::check_for_DZ(Track& tr)
{
    bool state = true;
        //! model each link as an elipsoid
        //! check for defined danger zones
        //! find some optimization method to limit calculations
    return state;
};

bool RuckigTrajectory::check_kinematic_limits(Track& tr){

    bool state=true;
    return state;

};


void RuckigTrajectory::rot_interpolate(Track& tr){

   Eigen::Quaterniond q_i(this->actual_pose.translation().matrix());
   // Eigen::Quaterniond q_f(this->final_pose.translation().matrix());
   // Eigen::Quaterniond q_t;

    //for (int i=0; i<tr.pose.size();i++){
    //q_t = q_i.slerp(.01, q_f);
    //tr.pose[i].linear().matrix()=q_t.toRotationMatrix();
    //};
    };


    //Eigen::Quaterniond q1(1, 0, 0, 0);
    //Eigen::Quaterniond q2(0.5, 0.5, 0.5, 0.5);
    //// Interpolate between the two quaternions
    //double t = 0.5; // interpolation factor
    //Eigen::Quaterniond q = q1.slerp(t, q2);
    //// Print the interpolated quaternion
    //std::cout << "Interpolated quaternion: " << q.coeffs().transpose() << std::endl;



    /// Eigen::Matrix3d rotation_matrix;
    //rotation_matrix << 0.707, -0.707, 0,
    //                  0.707, 0.707, 0,
    //                 0, 0, 1;
    // Convert the rotation matrix to a quaternion
    //Eigen::Quaterniond quaternion(rotation_matrix);
    // Print the quaternion
    //std::cout << "Quaternion: " << quaternion.coeffs().transpose() << std::endl;



    //Eigen::Quaterniond quaternion(0.5, 0.5, 0.5, 0.5);
    // Convert the quaternion to a rotation matrix
    //   Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
    // Print the rotation matrix
    // std::cout << "Rotation matrix: " << std::endl << rotation_matrix << std::endl;





//void recalculate(Track& tr){
//};
