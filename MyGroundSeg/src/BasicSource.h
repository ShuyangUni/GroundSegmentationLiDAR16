#pragma once
#ifndef TESTPCL_BS_H
#define TESTPCL_BS_H

/**
 * Include List
 */
#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include <Eigen/Dense>
#include <Eigen/Cholesky> 

using namespace std;

/**
 * Macro List
 */

/**
 * Const List
 */
// LiDAR info
const double k_PI = 3.1415926535;
const double k_VertAngRes_16 = 2.0 / 180 * k_PI;
const double k_HoriAngRes_16 = (360 / 2040) / 180 * k_PI;

const double k_HeightLiDAR = 1.65;

/**
 * Stuct
 */
struct PtPolar
{
    double m_Dis;
    double m_Angle;
    double m_height;
    int m_PtInd;

    PtPolar(double dis, double angle, double height, int ind)
    {
        m_Dis = dis;
        m_Angle = angle;
        m_height = height;
        m_PtInd = ind;
    }

    PtPolar(pcl::PointXYZ pcPtXYZ, int ind)
    {
        m_Dis = sqrt(pow(pcPtXYZ.x, 2) + pow(pcPtXYZ.y, 2));
        m_Angle = atan2(pcPtXYZ.y, pcPtXYZ.x);
        m_height = pcPtXYZ.z;
        m_PtInd = ind;
        if (m_Angle < 0)
        {
            m_Angle = m_Angle + k_PI*2;
        }
    }
};

// end of H_BS_H
#endif