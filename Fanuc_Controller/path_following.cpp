
#include <iostream>
#include<vector>

class path
{
public:
    ControlRobot();
    ~ControlRobot();

    
private:

     T01;
    Eigen::Matrix4f T12;
    Eigen::Matrix4f T23;
    Eigen::Matrix4f T34;
    Eigen::Matrix4f T45;
    Eigen::Matrix4f T56;
    Eigen::Matrix4f TransformationMatrice;
    Eigen::Matrix<float, 6, 6> JacobienneMatrice;
    Eigen::Matrix<float, 6, 6> JacobienneInverse;
    Eigen::Matrix<float, 6, 1> InverseKinematicResultats;
    Eigen::Matrix<float, 6, 1> JointPosition;

    int NumSphere;
};

#endif // CONTROLROBOT_H
