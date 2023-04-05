#include "ControlRobot.h"

#include <iostream>
#include <limits>

#define M_PI 3.14159265358979323846

ControlRobot::ControlRobot()
{
    this->T01 = Eigen::Matrix4f::Zero();
    this->T12 = Eigen::Matrix4f::Zero();
    this->T23 = Eigen::Matrix4f::Zero();
    this->T34 = Eigen::Matrix4f::Zero();
    this->T45 = Eigen::Matrix4f::Zero();
    this->T56 = Eigen::Matrix4f::Zero();
    this->TransformationMatrice = Eigen::Matrix4f::Zero();
    this->JacobienneMatrice = Eigen::Matrix<float, 6, 6>::Zero();
    this->JacobienneInverse = Eigen::Matrix<float, 6, 6>::Zero();
    this->InverseKinematicResultats = Eigen::Matrix<float, 6, 1>::Zero();
    this->JointPosition = Eigen::Matrix<float, 6, 1>::Zero();
    this->NumSphere = 0;
}

ControlRobot::~ControlRobot()
{

}

void ControlRobot::MatriceTransformationDHUR10(float &Joint1Position, float &Joint2Position, float &Joint3Position, float &Joint4Position, float &Joint5Position, float &Joint6Position)
{
    this->JointPosition(0, 0) = Joint1Position;
    this->JointPosition(1, 0) = Joint2Position;
    this->JointPosition(2, 0) = Joint3Position;
    this->JointPosition(3, 0) = Joint4Position;
    this->JointPosition(4, 0) = Joint5Position;
    this->JointPosition(5, 0) = Joint6Position;

    // T01

    this->T01(0, 0) = cos(this->JointPosition(0, 0) - M_PI);
    this->T01(0, 1) = -sin(this->JointPosition(0, 0) - M_PI) * cos(M_PI / 2);
    this->T01(0, 2) = sin(this->JointPosition(0, 0) - M_PI) * sin(M_PI / 2);
    this->T01(0, 3) = 0.0 * cos(this->JointPosition(0, 0) - M_PI);
    this->T01(1, 0) = sin(this->JointPosition(0, 0) - M_PI);
    this->T01(1, 1) = cos(this->JointPosition(0, 0) - M_PI) * cos(M_PI / 2);
    this->T01(1, 2) = -cos(this->JointPosition(0, 0) - M_PI) * sin(M_PI / 2);
    this->T01(1, 3) = 0.0 * sin(this->JointPosition(0, 0) - M_PI);
    this->T01(2, 0) = 0.0;
    this->T01(2, 1) = sin(M_PI / 2);
    this->T01(2, 2) = cos(M_PI / 2);
    this->T01(2, 3) = 0.1273;
    this->T01(3, 0) = 0.0;
    this->T01(3, 1) = 0.0;
    this->T01(3, 2) = 0.0;
    this->T01(3, 3) = 1.0;

    // T12

    this->T12(0, 0) = cos(this->JointPosition(1, 0) - (M_PI / 2));
    this->T12(0, 1) = -sin(this->JointPosition(1, 0) - (M_PI / 2)) * cos(0.0);
    this->T12(0, 2) = sin(this->JointPosition(1, 0) - (M_PI / 2)) * sin(0.0);
    this->T12(0, 3) = -0.612 * cos(this->JointPosition(1, 0) - (M_PI / 2));
    this->T12(1, 0) = sin(this->JointPosition(1, 0) - (M_PI / 2));
    this->T12(1, 1) = cos(this->JointPosition(1, 0) - (M_PI / 2)) * cos(0.0);
    this->T12(1, 2) = -cos(this->JointPosition(1, 0) - (M_PI / 2)) * sin(0.0);
    this->T12(1, 3) = -0.612 * sin(this->JointPosition(1, 0) - (M_PI / 2));
    this->T12(2, 0) = 0.0;
    this->T12(2, 1) = sin(0.0);
    this->T12(2, 2) = cos(0.0);
    this->T12(2, 3) = 0.0;
    this->T12(3, 0) = 0.0;
    this->T12(3, 1) = 0.0;
    this->T12(3, 2) = 0.0;
    this->T12(3, 3) = 1.0;

    // T23

    this->T23(0, 0) = cos(this->JointPosition(2, 0));
    this->T23(0, 1) = -sin(this->JointPosition(2, 0)) * cos(0.0);
    this->T23(0, 2) = sin(this->JointPosition(2, 0)) * sin(0.0);
    this->T23(0, 3) = -0.5723 * cos(this->JointPosition(2, 0));
    this->T23(1, 0) = sin(this->JointPosition(2, 0));
    this->T23(1, 1) = cos(this->JointPosition(2, 0)) * cos(0.0);
    this->T23(1, 2) = -cos(this->JointPosition(2, 0)) * sin(0.0);
    this->T23(1, 3) = -0.5723 * sin(this->JointPosition(2, 0));
    this->T23(2, 0) = 0.0;
    this->T23(2, 1) = sin(0.0);
    this->T23(2, 2) = cos(0.0);
    this->T23(2, 3) = 0.0;
    this->T23(3, 0) = 0.0;
    this->T23(3, 1) = 0.0;
    this->T23(3, 2) = 0.0;
    this->T23(3, 3) = 1.0;

    // T34

    this->T34(0, 0) = cos(this->JointPosition(3, 0) - (M_PI / 2));
    this->T34(0, 1) = -sin(this->JointPosition(3, 0) - (M_PI / 2)) * cos(M_PI / 2);
    this->T34(0, 2) = sin(this->JointPosition(3, 0) - (M_PI / 2)) * sin(M_PI / 2);
    this->T34(0, 3) = 0.0 * cos(this->JointPosition(3, 0) - (M_PI / 2));
    this->T34(1, 0) = sin(this->JointPosition(3, 0) - (M_PI / 2));
    this->T34(1, 1) = cos(this->JointPosition(3, 0) - (M_PI / 2)) * cos(M_PI / 2);
    this->T34(1, 2) = -cos(this->JointPosition(3, 0) - (M_PI / 2)) * sin(M_PI / 2);
    this->T34(1, 3) = 0.0 * sin(this->JointPosition(3, 0) - (M_PI / 2));
    this->T34(2, 0) = 0.0;
    this->T34(2, 1) = sin(M_PI / 2);
    this->T34(2, 2) = cos(M_PI / 2);
    this->T34(2, 3) = 0.163941;
    this->T34(3, 0) = 0.0;
    this->T34(3, 1) = 0.0;
    this->T34(3, 2) = 0.0;
    this->T34(3, 3) = 1.0;

    // T45

    this->T45(0, 0) = cos(this->JointPosition(4, 0));
    this->T45(0, 1) = -sin(this->JointPosition(4, 0)) * cos(-M_PI / 2);
    this->T45(0, 2) = sin(this->JointPosition(4, 0)) * sin(-M_PI / 2);
    this->T45(0, 3) = 0.0 * cos(this->JointPosition(4, 0));
    this->T45(1, 0) = sin(this->JointPosition(4, 0));
    this->T45(1, 1) = cos(this->JointPosition(4, 0)) * cos(-M_PI / 2);
    this->T45(1, 2) = -cos(this->JointPosition(4, 0)) * sin(-M_PI / 2);
    this->T45(1, 3) = 0.0 * sin(this->JointPosition(4, 0));
    this->T45(2, 0) = 0.0;
    this->T45(2, 1) = sin(-M_PI / 2);
    this->T45(2, 2) = cos(-M_PI / 2);
    this->T45(2, 3) = 0.1157;
    this->T45(3, 0) = 0.0;
    this->T45(3, 1) = 0.0;
    this->T45(3, 2) = 0.0;
    this->T45(3, 3) = 1.0;

    // T56

    this->T56(0, 0) = cos(this->JointPosition(5, 0));
    this->T56(0, 1) = -sin(this->JointPosition(5, 0)) * cos(0.0);
    this->T56(0, 2) = sin(this->JointPosition(5, 0)) * sin(0.0);
    this->T56(0, 3) = 0.0 * cos(this->JointPosition(5, 0));
    this->T56(1, 0) = sin(this->JointPosition(5, 0));
    this->T56(1, 1) = cos(this->JointPosition(5, 0)) * cos(0.0);
    this->T56(1, 2) = -cos(this->JointPosition(5, 0)) * sin(0.0);
    this->T56(1, 3) = 0.0 * sin(this->JointPosition(5, 0));
    this->T56(2, 0) = 0.0;
    this->T56(2, 1) = sin(0.0);
    this->T56(2, 2) = cos(0.0);
    this->T56(2, 3) = 0.0922;
    this->T56(3, 0) = 0.0;
    this->T56(3, 1) = 0.0;
    this->T56(3, 2) = 0.0;
    this->T56(3, 3) = 1.0;

    this->TransformationMatrice = this->T01 * this->T12 * this->T23 * this->T34 * this->T45 * this->T56;
}

void ControlRobot::MatriceTransformationDHUR10e(float &Joint1Position, float &Joint2Position, float &Joint3Position, float &Joint4Position, float &Joint5Position, float &Joint6Position)
{
    this->JointPosition(0, 0) = Joint1Position;
    this->JointPosition(1, 0) = Joint2Position;
    this->JointPosition(2, 0) = Joint3Position;
    this->JointPosition(3, 0) = Joint4Position;
    this->JointPosition(4, 0) = Joint5Position;
    this->JointPosition(5, 0) = Joint6Position;

    // T01

    this->T01(0, 0) = cos(this->JointPosition(0, 0));
    this->T01(0, 1) = -cos(M_PI / 2) * sin(this->JointPosition(0, 0));
    this->T01(0, 2) = sin(M_PI / 2) * sin(this->JointPosition(0, 0));
    this->T01(0, 3) = -0.f * cos(this->JointPosition(0, 0));
    this->T01(1, 0) = sin(this->JointPosition(0, 0));
    this->T01(1, 1) = cos(M_PI / 2) * cos(this->JointPosition(0, 0));
    this->T01(1, 2) = -sin(M_PI / 2) * cos(this->JointPosition(0, 0));
    this->T01(1, 3) = -0.f * sin(this->JointPosition(0, 0));
    this->T01(2, 0) = 0.f;
    this->T01(2, 1) = sin(M_PI / 2);
    this->T01(2, 2) = cos(M_PI / 2);
    this->T01(2, 3) = 0.1807;
    this->T01(3, 0) = 0.f;
    this->T01(3, 1) = 0.f;
    this->T01(3, 2) = 0.f;
    this->T01(3, 3) = 1.f;

    // T12

    this->T12(0, 0) = cos(this->JointPosition(1, 0));
    this->T12(0, 1) = -cos(0.f) * sin(this->JointPosition(1, 0));
    this->T12(0, 2) = sin(0.f) * sin(this->JointPosition(1, 0));
    this->T12(0, 3) = -0.6127 * cos(this->JointPosition(1, 0));
    this->T12(1, 0) = sin(this->JointPosition(1, 0));
    this->T12(1, 1) = cos(0.f) * cos(this->JointPosition(1, 0));
    this->T12(1, 2) = -sin(0.f) * cos(this->JointPosition(1, 0));
    this->T12(1, 3) = -0.6127 * sin(this->JointPosition(1, 0));
    this->T12(2, 0) = 0.f;
    this->T12(2, 1) = sin(0.f);
    this->T12(2, 2) = cos(0.f);
    this->T12(2, 3) = 0.f;
    this->T12(3, 0) = 0.f;
    this->T12(3, 1) = 0.f;
    this->T12(3, 2) = 0.f;
    this->T12(3, 3) = 1.f;

    // T23

    this->T23(0, 0) = cos(this->JointPosition(2, 0));
    this->T23(0, 1) = -cos(0.f) * sin(this->JointPosition(2, 0));
    this->T23(0, 2) = sin(0.f) * sin(this->JointPosition(2, 0));
    this->T23(0, 3) = -0.57155 * cos(this->JointPosition(2, 0));
    this->T23(1, 0) = sin(this->JointPosition(2, 0));
    this->T23(1, 1) = cos(0.f) * cos(this->JointPosition(2, 0));
    this->T23(1, 2) = -sin(0.f) * cos(this->JointPosition(2, 0));
    this->T23(1, 3) = -0.57155 * sin(this->JointPosition(2, 0));
    this->T23(2, 0) = 0.f;
    this->T23(2, 1) = sin(0.f);
    this->T23(2, 2) = cos(0.f);
    this->T23(2, 3) = 0.f;
    this->T23(3, 0) = 0.f;
    this->T23(3, 1) = 0.f;
    this->T23(3, 2) = 0.f;
    this->T23(3, 3) = 1.f;

    // T34

    this->T34(0, 0) = cos(this->JointPosition(3, 0));
    this->T34(0, 1) = -cos(M_PI / 2) * sin(this->JointPosition(3, 0));
    this->T34(0, 2) = sin(M_PI / 2) * sin(this->JointPosition(3, 0));
    this->T34(0, 3) = 0.f * cos(this->JointPosition(3, 0));
    this->T34(1, 0) = sin(this->JointPosition(3, 0));
    this->T34(1, 1) = cos(M_PI / 2) * cos(this->JointPosition(3, 0));
    this->T34(1, 2) = -sin(M_PI / 2) * cos(this->JointPosition(3, 0));
    this->T34(1, 3) = 0.f * sin(this->JointPosition(3, 0));
    this->T34(2, 0) = 0.f;
    this->T34(2, 1) = sin(M_PI / 2);
    this->T34(2, 2) = cos(M_PI / 2);
    this->T34(2, 3) = 0.17415;
    this->T34(3, 0) = 0.f;
    this->T34(3, 1) = 0.f;
    this->T34(3, 2) = 0.f;
    this->T34(3, 3) = 1.f;

    // T45

    this->T45(0, 0) = cos(this->JointPosition(4, 0));
    this->T45(0, 1) = -cos(-M_PI / 2) * sin(this->JointPosition(4, 0));
    this->T45(0, 2) = sin(-M_PI / 2) * sin(this->JointPosition(4, 0));
    this->T45(0, 3) = 0.f * cos(this->JointPosition(4, 0));
    this->T45(1, 0) = sin(this->JointPosition(4, 0));
    this->T45(1, 1) = cos(-M_PI / 2) * cos(this->JointPosition(4, 0));
    this->T45(1, 2) = -sin(-M_PI / 2) * cos(this->JointPosition(4, 0));
    this->T45(1, 3) = 0.f * sin(this->JointPosition(4, 0));
    this->T45(2, 0) = 0.f;
    this->T45(2, 1) = sin(-M_PI / 2);
    this->T45(2, 2) = cos(-M_PI / 2);
    this->T45(2, 3) = 0.11985;
    this->T45(3, 0) = 0.f;
    this->T45(3, 1) = 0.f;
    this->T45(3, 2) = 0.f;
    this->T45(3, 3) = 1.f;

    // T56

    this->T56(0, 0) = cos(this->JointPosition(5, 0));
    this->T56(0, 1) = -cos(0.f) * sin(this->JointPosition(5, 0));
    this->T56(0, 2) = sin(0.f) * sin(this->JointPosition(5, 0));
    this->T56(0, 3) = 0.f * cos(this->JointPosition(5, 0));
    this->T56(1, 0) = sin(this->JointPosition(5, 0));
    this->T56(1, 1) = cos(0.f) * cos(this->JointPosition(5, 0));
    this->T56(1, 2) = -sin(0.f) * cos(this->JointPosition(5, 0));
    this->T56(1, 3) = 0.f * sin(this->JointPosition(5, 0));
    this->T56(2, 0) = 0.f;
    this->T56(2, 1) = sin(0.f);
    this->T56(2, 2) = cos(0.f);
    this->T56(2, 3) = 0.11655;
    this->T56(3, 0) = 0.f;
    this->T56(3, 1) = 0.f;
    this->T56(3, 2) = 0.f;
    this->T56(3, 3) = 1.f;

    this->TransformationMatrice = this->T01 * this->T12 * this->T23 * this->T34 * this->T45 * this->T56;
}

void ControlRobot::MatriceTransformationDHFanuc(float &Joint1Position, float &Joint2Position, float &Joint3Position, float &Joint4Position, float &Joint5Position, float &Joint6Position)
{
    Eigen::Matrix<float, 6, 4> DH = Eigen::Matrix<float, 6, 4>::Zero();
    // a(degré)                 a(metre) ou r          o(oméga)                                                  d
    DH(0, 0) = -M_PI / 2;       DH(0, 1) = 0.075;      DH(0, 2) = Joint1Position + (0.f * M_PI / 180);           DH(0, 3) = 0.648;
    DH(1, 0) = 0.f;             DH(1, 1) = 0.640;      DH(1, 2) = Joint2Position + (-90.f * M_PI / 180);         DH(1, 3) = 0.f;
    DH(2, 0) = -M_PI / 2;       DH(2, 1) = 0.195;      DH(2, 2) = Joint3Position + (0.f * M_PI / 180);           DH(2, 3) = 0.f;
    DH(3, 0) = -M_PI / 2;       DH(3, 1) = 0.f;        DH(3, 2) = Joint4Position + (-180.f * M_PI / 180);        DH(3, 3) = 0.700;
    DH(4, 0) = -M_PI / 2;       DH(4, 1) = 0.f;        DH(4, 2) = Joint5Position + (-180.f * M_PI / 180);        DH(4, 3) = 0.f;
    DH(5, 0) = 0.f;             DH(5, 1) = 0.f;        DH(5, 2) = Joint6Position + (0.f * M_PI / 180);           DH(5, 3) = 0.075;

    // T01

    this->T01(0, 0) = cos(DH(0, 2));
    this->T01(0, 1) = -cos(DH(0, 0)) * sin(DH(0, 2));
    this->T01(0, 2) = sin(DH(0, 0)) * sin(DH(0, 2));
    this->T01(0, 3) = DH(0, 1) * cos(DH(0, 2));
    this->T01(1, 0) = sin(DH(0, 2));
    this->T01(1, 1) = cos(DH(0, 0)) * cos(DH(0, 2));
    this->T01(1, 2) = -sin(DH(0, 0)) * cos(DH(0, 2));
    this->T01(1, 3) = DH(0, 1) * sin(DH(0, 2));
    this->T01(2, 0) = 0.f;
    this->T01(2, 1) = sin(DH(0, 0));
    this->T01(2, 2) = cos(DH(0, 0));
    this->T01(2, 3) = DH(0, 3);
    this->T01(3, 0) = 0.f;
    this->T01(3, 1) = 0.f;
    this->T01(3, 2) = 0.f;
    this->T01(3, 3) = 1.f;

    // T12

    this->T12(0, 0) = cos(DH(1, 2));
    this->T12(0, 1) = -cos(DH(1, 0)) * sin(DH(1, 2));
    this->T12(0, 2) = sin(DH(1, 0)) * sin(DH(1, 2));
    this->T12(0, 3) = DH(1, 1) * cos(DH(1, 2));
    this->T12(1, 0) = sin(DH(1, 2));
    this->T12(1, 1) = cos(DH(1, 0)) * cos(DH(1, 2));
    this->T12(1, 2) = -sin(DH(1, 0)) * cos(DH(1, 2));
    this->T12(1, 3) = DH(1, 1) * sin(DH(1, 2));
    this->T12(2, 0) = 0.f;
    this->T12(2, 1) = sin(DH(1, 0));
    this->T12(2, 2) = cos(DH(1, 0));
    this->T12(2, 3) = DH(1, 3);
    this->T12(3, 0) = 0.f;
    this->T12(3, 1) = 0.f;
    this->T12(3, 2) = 0.f;
    this->T12(3, 3) = 1.f;

    // T23

    this->T23(0, 0) = cos(DH(2, 2));
    this->T23(0, 1) = -cos(DH(2, 0)) * sin(DH(2, 2));
    this->T23(0, 2) = sin(DH(2, 0)) * sin(DH(2, 2));
    this->T23(0, 3) = DH(2, 1) * cos(DH(2, 2));
    this->T23(1, 0) = sin(DH(2, 2));
    this->T23(1, 1) = cos(DH(2, 0)) * cos(DH(2, 2));
    this->T23(1, 2) = -sin(DH(2, 0)) * cos(DH(2, 2));
    this->T23(1, 3) = DH(2, 1) * sin(DH(2, 2));
    this->T23(2, 0) = 0.f;
    this->T23(2, 1) = sin(DH(2, 0));
    this->T23(2, 2) = cos(DH(2, 0));
    this->T23(2, 3) = DH(2, 3);
    this->T23(3, 0) = 0.f;
    this->T23(3, 1) = 0.f;
    this->T23(3, 2) = 0.f;
    this->T23(3, 3) = 1.f;

    // T34

    this->T34(0, 0) = cos(DH(3, 2));
    this->T34(0, 1) = -cos(DH(3, 0)) * sin(DH(3, 2));
    this->T34(0, 2) = sin(DH(3, 0)) * sin(DH(3, 2));
    this->T34(0, 3) = DH(3, 1) * cos(DH(3, 2));
    this->T34(1, 0) = sin(DH(3, 2));
    this->T34(1, 1) = cos(DH(3, 0)) * cos(DH(3, 2));
    this->T34(1, 2) = -sin(DH(3, 0)) * cos(DH(3, 2));
    this->T34(1, 3) = DH(3, 1) * sin(DH(3, 2));
    this->T34(2, 0) = 0.f;
    this->T34(2, 1) = sin(DH(3, 0));
    this->T34(2, 2) = cos(DH(3, 0));
    this->T34(2, 3) = DH(3, 3);
    this->T34(3, 0) = 0.f;
    this->T34(3, 1) = 0.f;
    this->T34(3, 2) = 0.f;
    this->T34(3, 3) = 1.f;

    // T45

    this->T45(0, 0) = cos(DH(4, 2));
    this->T45(0, 1) = -cos(DH(4, 0)) * sin(DH(4, 2));
    this->T45(0, 2) = sin(DH(4, 0)) * sin(DH(4, 2));
    this->T45(0, 3) = DH(4, 1) * cos(DH(4, 2));
    this->T45(1, 0) = sin(DH(4, 2));
    this->T45(1, 1) = cos(DH(4, 0)) * cos(DH(4, 2));
    this->T45(1, 2) = -sin(DH(4, 0)) * cos(DH(4, 2));
    this->T45(1, 3) = DH(4, 1) * sin(DH(4, 2));
    this->T45(2, 0) = 0.f;
    this->T45(2, 1) = sin(DH(4, 0));
    this->T45(2, 2) = cos(DH(4, 0));
    this->T45(2, 3) = DH(4, 3);
    this->T45(3, 0) = 0.f;
    this->T45(3, 1) = 0.f;
    this->T45(3, 2) = 0.f;
    this->T45(3, 3) = 1.f;

    // T56

    this->T56(0, 0) = cos(DH(5, 2));
    this->T56(0, 1) = -cos(DH(5, 0)) * sin(DH(5, 2));
    this->T56(0, 2) = sin(DH(5, 0)) * sin(DH(5, 2));
    this->T56(0, 3) = DH(5, 1) * cos(DH(5, 2));
    this->T56(1, 0) = sin(DH(5, 2));
    this->T56(1, 1) = cos(DH(5, 0)) * cos(DH(5, 2));
    this->T56(1, 2) = -sin(DH(5, 0)) * cos(DH(5, 2));
    this->T56(1, 3) = DH(5, 1) * sin(DH(5, 2));
    this->T56(2, 0) = 0.f;
    this->T56(2, 1) = sin(DH(5, 0));
    this->T56(2, 2) = cos(DH(5, 0));
    this->T56(2, 3) = DH(5, 3);
    this->T56(3, 0) = 0.f;
    this->T56(3, 1) = 0.f;
    this->T56(3, 2) = 0.f;
    this->T56(3, 3) = 1.f;

    this->TransformationMatrice = this->T01 * this->T12 * this->T23 * this->T34 * this->T45 * this->T56;
}

void ControlRobot::MatriceJacobienne()
{
    Eigen::Matrix4f T02;
    Eigen::Matrix4f T03;
    Eigen::Matrix4f T04;
    Eigen::Matrix4f T05;
    Eigen::Matrix4f T06;

    Eigen::Matrix<float, 3, 1> Matrice001;
    Eigen::Matrix3f Rotation;
    Eigen::Matrix<float, 3, 1> DistanceArriver;
    Eigen::Matrix<float, 3, 1> DistanceDepart;
    Eigen::Matrix<float, 3, 1> Resultat1;
    Eigen::Matrix<float, 3, 1> Resultat2;
    Eigen::Matrix<float, 3, 1> Resultat;

    T02 = this->T01 * this->T12;
    T03 = T02 * this->T23;
    T04 = T03 * this->T34;
    T05 = T04 * this->T45;
    T06 = T05 * this->T56;

    Matrice001(0, 0) = 0.0;
    Matrice001(1, 0) = 0.0;
    Matrice001(2, 0) = 1.0;

    DistanceArriver(0, 0) = T06(0, 3);
    DistanceArriver(1, 0) = T06(1, 3);
    DistanceArriver(2, 0) = T06(2, 3);

    // 1
    Resultat1 = Matrice001;
    Resultat2 = DistanceArriver;
    Resultat(0, 0) = (Resultat1(1, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(1, 0));
    Resultat(1, 0) = ((Resultat1(0, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(0, 0))) * -1;
    Resultat(2, 0) = (Resultat1(0, 0) * Resultat2(1, 0)) - (Resultat1(1, 0) * Resultat2(0, 0));

    this->JacobienneMatrice(0, 0) = Resultat(0, 0);
    this->JacobienneMatrice(1, 0) = Resultat(1, 0);
    this->JacobienneMatrice(2, 0) = Resultat(2, 0);

    this->JacobienneMatrice(3, 0) = Resultat1(0, 0);
    this->JacobienneMatrice(4, 0) = Resultat1(1, 0);
    this->JacobienneMatrice(5, 0) = Resultat1(2, 0);

    // 2
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            Rotation(i, j) = this->T01(i, j);
        }

        DistanceDepart(i, 0) = T01(i, 3);
    }
    Resultat1 = Rotation * Matrice001;
    Resultat2 = DistanceArriver - DistanceDepart;
    Resultat(0, 0) = (Resultat1(1, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(1, 0));
    Resultat(1, 0) = ((Resultat1(0, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(0, 0))) * -1;
    Resultat(2, 0) = (Resultat1(0, 0) * Resultat2(1, 0)) - (Resultat1(1, 0) * Resultat2(0, 0));

    this->JacobienneMatrice(0, 1) = Resultat(0, 0);
    this->JacobienneMatrice(1, 1) = Resultat(1, 0);
    this->JacobienneMatrice(2, 1) = Resultat(2, 0);

    this->JacobienneMatrice(3, 1) = Resultat1(0, 0);
    this->JacobienneMatrice(4, 1) = Resultat1(1, 0);
    this->JacobienneMatrice(5, 1) = Resultat1(2, 0);

    // 3
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            Rotation(i, j) = T02(i, j);
        }

        DistanceDepart(i, 0) = T02(i, 3);
    }
    Resultat1 = Rotation * Matrice001;
    Resultat2 = DistanceArriver - DistanceDepart;
    Resultat(0, 0) = (Resultat1(1, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(1, 0));
    Resultat(1, 0) = ((Resultat1(0, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(0, 0))) * -1;
    Resultat(2, 0) = (Resultat1(0, 0) * Resultat2(1, 0)) - (Resultat1(1, 0) * Resultat2(0, 0));

    this->JacobienneMatrice(0, 2) = Resultat(0, 0);
    this->JacobienneMatrice(1, 2) = Resultat(1, 0);
    this->JacobienneMatrice(2, 2) = Resultat(2, 0);

    this->JacobienneMatrice(3, 2) = Resultat1(0, 0);
    this->JacobienneMatrice(4, 2) = Resultat1(1, 0);
    this->JacobienneMatrice(5, 2) = Resultat1(2, 0);

    // 4
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            Rotation(i, j) = T03(i, j);
        }

        DistanceDepart(i, 0) = T03(i, 3);
    }
    Resultat1 = Rotation * Matrice001;
    Resultat2 = DistanceArriver - DistanceDepart;
    Resultat(0, 0) = (Resultat1(1, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(1, 0));
    Resultat(1, 0) = ((Resultat1(0, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(0, 0))) * -1;
    Resultat(2, 0) = (Resultat1(0, 0) * Resultat2(1, 0)) - (Resultat1(1, 0) * Resultat2(0, 0));

    this->JacobienneMatrice(0, 3) = Resultat(0, 0);
    this->JacobienneMatrice(1, 3) = Resultat(1, 0);
    this->JacobienneMatrice(2, 3) = Resultat(2, 0);

    this->JacobienneMatrice(3, 3) = Resultat1(0, 0);
    this->JacobienneMatrice(4, 3) = Resultat1(1, 0);
    this->JacobienneMatrice(5, 3) = Resultat1(2, 0);

    // 5
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            Rotation(i, j) = T04(i, j);
        }

        DistanceDepart(i, 0) = T04(i, 3);
    }
    Resultat1 = Rotation * Matrice001;
    Resultat2 = DistanceArriver - DistanceDepart;
    Resultat(0, 0) = (Resultat1(1, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(1, 0));
    Resultat(1, 0) = ((Resultat1(0, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(0, 0))) * -1;
    Resultat(2, 0) = (Resultat1(0, 0) * Resultat2(1, 0)) - (Resultat1(1, 0) * Resultat2(0, 0));

    this->JacobienneMatrice(0, 4) = Resultat(0, 0);
    this->JacobienneMatrice(1, 4) = Resultat(1, 0);
    this->JacobienneMatrice(2, 4) = Resultat(2, 0);

    this->JacobienneMatrice(3, 4) = Resultat1(0, 0);
    this->JacobienneMatrice(4, 4) = Resultat1(1, 0);
    this->JacobienneMatrice(5, 4) = Resultat1(2, 0);

    // 6
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            Rotation(i, j) = T05(i, j);
        }

        DistanceDepart(i, 0) = T05(i, 3);
    }
    Resultat1 = Rotation * Matrice001;
    Resultat2 = DistanceArriver - DistanceDepart;
    Resultat(0, 0) = (Resultat1(1, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(1, 0));
    Resultat(1, 0) = ((Resultat1(0, 0) * Resultat2(2, 0)) - (Resultat1(2, 0) * Resultat2(0, 0))) * -1;
    Resultat(2, 0) = (Resultat1(0, 0) * Resultat2(1, 0)) - (Resultat1(1, 0) * Resultat2(0, 0));

    this->JacobienneMatrice(0, 5) = Resultat(0, 0);
    this->JacobienneMatrice(1, 5) = Resultat(1, 0);
    this->JacobienneMatrice(2, 5) = Resultat(2, 0);

    this->JacobienneMatrice(3, 5) = Resultat1(0, 0);
    this->JacobienneMatrice(4, 5) = Resultat1(1, 0);
    this->JacobienneMatrice(5, 5) = Resultat1(2, 0);
}

void ControlRobot::MatriceJacobienneV2()
{
    Eigen::Matrix4f T02;
    Eigen::Matrix4f T03;
    Eigen::Matrix4f T04;
    Eigen::Matrix4f T05;
    Eigen::Matrix4f T06;

    Eigen::Matrix<float, 3, 1> Matrice001;
    Eigen::Matrix3f Rotation;
    Eigen::Matrix<float, 3, 1> DistanceArriver;
    Eigen::Matrix<float, 3, 1> DistanceDepart;
    Eigen::Matrix<float, 3, 1> Resultat1;
    Eigen::Matrix<float, 3, 1> Resultat2;
    Eigen::Matrix<float, 3, 1> Resultat;

    T02 = this->T01 * this->T12;
    T03 = T02 * this->T23;
    T04 = T03 * this->T34;
    T05 = T04 * this->T45;
    T06 = T05 * this->T56;

    Eigen::Matrix<float, 3, 1> z0;
        z0  << 0, 0, 1;

    // Ji linear velocity
    this->JacobienneMatrice.block<3,1>(0,0) = z0.cross(T06.block<3,1>(0,3));
    this->JacobienneMatrice.block<3,1>(0,1) = T01.block<3,1>(0,2).cross(T06.block<3,1>(0,3) - T01.block<3,1>(0,3));
    this->JacobienneMatrice.block<3,1>(0,2) = T02.block<3,1>(0,2).cross(T06.block<3,1>(0,3) - T02.block<3,1>(0,3));
    this->JacobienneMatrice.block<3,1>(0,3) = T03.block<3,1>(0,2).cross(T06.block<3,1>(0,3) - T03.block<3,1>(0,3));
    this->JacobienneMatrice.block<3,1>(0,4) = T04.block<3,1>(0,2).cross(T06.block<3,1>(0,3) - T04.block<3,1>(0,3));
    this->JacobienneMatrice.block<3,1>(0,5) = T05.block<3,1>(0,2).cross(T06.block<3,1>(0,3) - T05.block<3,1>(0,3));
}

void ControlRobot::MatriceJacobienneInverse()
{
    // Inverse
    this->JacobienneInverse = this->JacobienneMatrice.inverse();
}

void ControlRobot::MatriceJacobiennePseudoInverse()
{
    // Pseudo inverse
    this->JacobienneInverse = this->JacobienneMatrice.transpose() * (this->JacobienneMatrice * this->JacobienneMatrice.transpose()).inverse();

    Eigen::MatrixXf jacobienne = Eigen::MatrixXf(3, 6);

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            jacobienne(i, j) = this->JacobienneMatrice(i, j);
        }
    }

    Eigen::MatrixXf pinv = jacobienne.completeOrthogonalDecomposition().pseudoInverse();

    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            this->JacobienneInverse(i, j) = pinv(i, j);
        }
    }
}







Eigen::Matrix<float, 6, 1> ControlRobot::InverseKinematic()
{
    Eigen::Matrix<float, 3, 1> PositionVoulue;
    Eigen::Matrix<float, 6, 1> Deplacement;

    switch(this->NumSphere)
    {
        case 0:
        {
            PositionVoulue(0, 0) = 0.85;
            PositionVoulue(1, 0) = 0.f;
            PositionVoulue(2, 0) = 1.483;

            float DistanceDeplacement = std::sqrt(std::pow(PositionVoulue(0, 0) - TransformationMatrice(0, 3),2) + std::pow(PositionVoulue(1, 0) - TransformationMatrice(1, 3),2) + std::pow(PositionVoulue(2, 0) - TransformationMatrice(2, 3),2));

            std::cout << "DistanceDeplacement 0" << std::endl;
            std::cout << DistanceDeplacement << std::endl;

            if(DistanceDeplacement <= 0.001)
            {
                this->NumSphere = 1;
            }

            break;
        }
        case 1:
        {
            PositionVoulue(0, 0) = 0.f;
            PositionVoulue(1, 0) = -0.85;
            PositionVoulue(2, 0) = 0.483;

            float DistanceDeplacement = std::sqrt(std::pow(PositionVoulue(0, 0) - TransformationMatrice(0, 3),2) + std::pow(PositionVoulue(1, 0) - TransformationMatrice(1, 3),2) + std::pow(PositionVoulue(2, 0) - TransformationMatrice(2, 3),2));

            std::cout << "DistanceDeplacement 1" << std::endl;
            std::cout << DistanceDeplacement << std::endl;

            if(DistanceDeplacement <= 0.001)
            {
                this->NumSphere = 2;
            }

            break;
        }
        case 2:
        {
            PositionVoulue(0, 0) = -0.85;
            PositionVoulue(1, 0) = 0.f;
            PositionVoulue(2, 0) = 1.483;

            float DistanceDeplacement = std::sqrt(std::pow(PositionVoulue(0, 0) - TransformationMatrice(0, 3),2) + std::pow(PositionVoulue(1, 0) - TransformationMatrice(1, 3),2) + std::pow(PositionVoulue(2, 0) - TransformationMatrice(2, 3),2));

            std::cout << "DistanceDeplacement 2" << std::endl;
            std::cout << DistanceDeplacement << std::endl;

            if(DistanceDeplacement <= 0.001)
            {
                this->NumSphere = 3;
            }

            break;
        }
        case 3:
        {
            PositionVoulue(0, 0) = 0.f;
            PositionVoulue(1, 0) = 0.85;
            PositionVoulue(2, 0) = 0.483;

            float DistanceDeplacement = std::sqrt(std::pow(PositionVoulue(0, 0) - TransformationMatrice(0, 3),2) + std::pow(PositionVoulue(1, 0) - TransformationMatrice(1, 3),2) + std::pow(PositionVoulue(2, 0) - TransformationMatrice(2, 3),2));

            std::cout << "DistanceDeplacement 3" << std::endl;
            std::cout << DistanceDeplacement << std::endl;

            if(DistanceDeplacement <= 0.001)
            {
                this->NumSphere = 0;
            }

            break;
        }
        default:
        {
            this->NumSphere = 0;
            break;
        }
    }

    Deplacement(0, 0) = PositionVoulue(0, 0) - TransformationMatrice(0, 3);
    Deplacement(1, 0) = PositionVoulue(1, 0) - TransformationMatrice(1, 3);
    Deplacement(2, 0) = PositionVoulue(2, 0) - TransformationMatrice(2, 3);
    Deplacement(3, 0) = 0.f;
    Deplacement(4, 0) = 0.f;
    Deplacement(5, 0) = 0.f;

    // vitesse
    this->InverseKinematicResultats = JacobienneInverse * Deplacement;

    return this->InverseKinematicResultats;
}
