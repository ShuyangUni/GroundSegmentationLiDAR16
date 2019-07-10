#pragma once
#ifndef TESTPCL_GS_H
#define TESTPCL_GS_H

#include "BasicSource.h"
#include "Param.h"

class GroundSegmentor
{
public:
    GroundSegmentor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Param &param);
    GroundSegmentor(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Param &param);
    ~GroundSegmentor(){};

    void run();
    void output(vector<int> &vecGroundInd, vector<int> &vecNoGroundInd, vector<int> &vecOutlierInd);

private:
    /**
     * Parameters
     */
    // Para
    Param m_Param;

    // Index Vector
    vector<int> m_vecbGround; // ground = 0; noGround = 1; Outlier = -1;

    vector<int> m_vecGroundInd;
    vector<int> m_vecNoGroundInd;
    vector<int> m_vecOutlierInd;

    // PointCloud
    pcl::PointCloud<pcl::PointXYZ> m_Cloud;

    /**
     * Functions
     */
    void InitilizeIndex();

    // Data Treatment
    void PrunePtsbyVehicle();
    void PrunePtsbyHeight();
    void TransferPts2PolarSys(vector<PtPolar> &vecPtsPolar);

    // Ground Segmentation
    void GroundSegmentationwithFixedGPHyp(const vector<PtPolar> &vecPtsPolar);
    void SplitIntoSections(const vector<PtPolar> &vecPtsPolar,vector<vector<PtPolar>> &vvecPtsInSection);
    void TreatwithSections(vector<vector<PtPolar>> &vvecPtsInSection);
    void SortByDistanceAcsending(vector<PtPolar> &vecPtsSection);

    // Gaussian Process Regression
    void GroundPtsEvaluator(const vector<PtPolar> &vecCandidatePts, const vector<PtPolar> &vecTestingPts, 
                            vector<PtPolar> &vecGroundPts, vector<PtPolar> &vecNoGroundPts);
    void GPRPredictor(const Eigen::VectorXf &vTrainingX, const Eigen::VectorXf &vTrainingY,
                    const Eigen::VectorXf &vPredictingX, 
                    Eigen::VectorXf &vPredictingY, Eigen::VectorXf &vPredictingCov);
    void CalculateK(const Eigen::VectorXf &x, const Eigen::VectorXf &y, const pair<double,double> &cov, Eigen::MatrixXf &K);

    // Output
    void UpdateOutput();
};

#endif