#pragma once
#ifndef TESTPCL_PARAM_H
#define TESTPCL_PARAM_H

#include "BasicSource.h"

struct GPParam
{
    pair<double,double> m_Cov;
    pair<double,double> m_Mean;
    double m_Lik;
    double m_ThresholdCov;

    GPParam()
    {
        m_Cov.first = 6.0;
        m_Cov.second = 0.7071; 
        m_Mean.first = 0;
        m_Mean.second = -k_HeightLiDAR;
        m_Lik = 0.12;
        m_ThresholdCov = 2.0;

        m_Lik = log(m_Lik);
        m_Cov.first = log(m_Cov.first);
        m_Cov.second = log(m_Cov.second);
    }
};

struct Param
{
    double m_MaxPtHeight;
    double m_Height_LiDARtoGround;
    int m_NumSection;
    double m_AngleErrGround;
    double m_MaxDisPtGroundCandidate;
    int m_MinNumPtCandidate;

    // Vehicle Info
    double m_VehicleLeft;
    double m_VehicleRight;
    double m_VehicleFront;
    double m_VehicleRear;

    GPParam m_GPParam;

    Param()
    {
        m_Height_LiDARtoGround = k_HeightLiDAR;
        m_MaxPtHeight = 2.0 - m_Height_LiDARtoGround;

        m_NumSection = 360;
        m_AngleErrGround = 2.0 / 180 * k_PI;
        m_MaxDisPtGroundCandidate = 25.0;
        m_MinNumPtCandidate = 5;

        m_VehicleLeft = 0.7;
        m_VehicleRight = -0.7;
        m_VehicleFront = 0.6;
        m_VehicleRear = -2.3;
    }
};


#endif