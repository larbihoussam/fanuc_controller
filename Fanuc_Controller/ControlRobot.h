#ifndef CONTROLROBOT_H
#define CONTROLROBOT_H

#include "Eigen"


class ControlRobot
{
public:
    ControlRobot();
    ~ControlRobot();

    void MatriceTransformationDHUR10(float &Joint1Position, float &Joint2Position, float &Joint3Position, float &Joint4Position, float &Joint5Position, float &Joint6Position);
    void MatriceTransformationDHUR10e(float &Joint1Position, float &Joint2Position, float &Joint3Position, float &Joint4Position, float &Joint5Position, float &Joint6Position);
    void MatriceTransformationDHFanuc(float &Joint1Position, float &Joint2Position, float &Joint3Position, float &Joint4Position, float &Joint5Position, float &Joint6Position);
    void MatriceJacobienne();
    void MatriceJacobienneV2();
    void MatriceJacobienneInverse();
    void MatriceJacobiennePseudoInverse();
    Eigen::Matrix<float, 6, 1> InverseKinematic();

private:

    Eigen::Matrix4f T01;
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
