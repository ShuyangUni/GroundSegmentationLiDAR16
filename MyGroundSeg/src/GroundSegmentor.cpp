#include "GroundSegmentor.h"

/**
 * Constructor & Destructors
 */
GroundSegmentor::GroundSegmentor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Param &param)
{
    m_Cloud = *cloud;
    m_Param = param;

    m_vecGroundInd.clear();
    m_vecNoGroundInd.clear();
    m_vecOutlierInd.clear();

    InitilizeIndex();
}

GroundSegmentor::GroundSegmentor(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Param &param)
{
    m_Cloud = cloud;
    m_Param = param;

    m_vecGroundInd.clear();
    m_vecNoGroundInd.clear();
    m_vecOutlierInd.clear();

    InitilizeIndex();
}

/**
 * Public Function
 */

void GroundSegmentor::run()
{
    PrunePtsbyVehicle();
    PrunePtsbyHeight();
    // Test
    
    m_vecGroundInd.clear();
    for (int i = 0; i < m_vecbGround.size(); ++i)
    {
        if(m_vecbGround.at(i)!=-1)
        {
            m_vecGroundInd.push_back(i);
        }
    }

    // vector<PtPolar> vecPtsPolar;
    // TransferPts2PolarSys(vecPtsPolar);

    // vector<vector<PtPolar>> vvecPtsInSection;
    // SplitIntoSections(vecPtsPolar, vvecPtsInSection);

    // TreatwithSections(vvecPtsInSection);
    // UpdateOutput();
}

void GroundSegmentor::output(vector<int> &vecGroundInd, vector<int> &vecNoGroundInd, vector<int> &vecOutlierInd)
{
    vecGroundInd = m_vecGroundInd;
    vecNoGroundInd = m_vecNoGroundInd;
    vecOutlierInd = m_vecOutlierInd;
}

/**
 * Private Function
 */
void GroundSegmentor::InitilizeIndex()
{
    for (int i = 0; i < m_Cloud.points.size(); ++i)
    {
        if (isnan(m_Cloud.points[i].x))
        {
            m_vecbGround.push_back(-1);
        }
        else
        {
            m_vecbGround.push_back(0);
        }
    }
}

void GroundSegmentor::PrunePtsbyVehicle()
{
    for (int i = 0; i < m_Cloud.points.size(); ++i)
    {
        if (m_Cloud.points[i].y < m_Param.m_VehicleLeft
        && m_Cloud.points[i].y > m_Param.m_VehicleRight
        && m_Cloud.points[i].x < m_Param.m_VehicleFront
        && m_Cloud.points[i].x > m_Param.m_VehicleRear)
        {
            m_vecbGround.at(i) = -1;
        }
    }
}

void GroundSegmentor::PrunePtsbyHeight()
{
    for (int i = 0; i < m_Cloud.points.size(); ++i)
    {
        if (m_Cloud.points[i].z > m_Param.m_MaxPtHeight)
        {
            m_vecbGround.at(i) = -1;
        }
    }
}

void GroundSegmentor::TransferPts2PolarSys(vector<PtPolar> &vecPtsPolar)
{
    vecPtsPolar.clear();
    for (int i = 0; i < m_Cloud.points.size(); ++i)
    {
        if (m_vecbGround[i] != -1)
        {
            PtPolar pt(m_Cloud.at(i), i);
            vecPtsPolar.push_back(pt);
        }
    }
}

void GroundSegmentor::SplitIntoSections(const vector<PtPolar> &vecPtsPolar, vector<vector<PtPolar>> &vvecPtsInSection)
{
    // Init
    vvecPtsInSection.clear();
    for (int i = 0; i < m_Param.m_NumSection; ++i)
    {
        vector<PtPolar> tmpVecPt;
        tmpVecPt.clear();
        vvecPtsInSection.push_back(tmpVecPt);
    }

    // Allocate Pts
    double angleRes = k_PI * 2 / m_Param.m_NumSection;
    for (int i = 0; i < vecPtsPolar.size(); ++i)
    {
        int tmpidx = (int)(vecPtsPolar.at(i).m_Angle / angleRes);
        vvecPtsInSection.at(tmpidx).push_back(vecPtsPolar.at(i));
    }
}

void GroundSegmentor::TreatwithSections(vector<vector<PtPolar>> &vvecPtsInSection)
{
    for (int i = 0; i < vvecPtsInSection.size(); ++i)
    {
        vector<PtPolar> tmpvecPtsSection = vvecPtsInSection.at(i);

        // Sort by Distance
        SortByDistanceAcsending(tmpvecPtsSection);

        //Using Planar a prior find candidate pts
        double tanMax = tan(m_Param.m_AngleErrGround);
        double tanMin = tan(-m_Param.m_AngleErrGround);

        vector<PtPolar> vecCandidatePts;
        vecCandidatePts.clear();

        for (int iPt = 0; iPt < tmpvecPtsSection.size(); iPt++)
        {
            PtPolar tmpPt = tmpvecPtsSection.at(iPt);
            pair<double, double> Range(tanMin * tmpPt.m_Dis - m_Param.m_Height_LiDARtoGround,
                                       tanMax * tmpPt.m_Dis - m_Param.m_Height_LiDARtoGround);
            // Is Candidate GroundPt?
            if (tmpPt.m_height >= Range.first && tmpPt.m_height <= Range.second)
            {
                // Candidate GroundPt < maxCandidateDistance?
                if (tmpPt.m_Dis < m_Param.m_MaxDisPtGroundCandidate)
                {
                    vecCandidatePts.push_back(tmpPt);
                }
            }
        } // End of Candidate Pts in each Section

        // Judge Number of Candidate Pts
        if (vecCandidatePts.size() < m_Param.m_MinNumPtCandidate)
        {
            // Abandon this section
            for (int j = 0; j < tmpvecPtsSection.size(); ++j)
            {
                m_vecbGround.at(tmpvecPtsSection.at(j).m_PtInd) = 1;
            }
            for (int j = 0; j < vecCandidatePts.size(); ++j)
            {
                m_vecbGround.at(vecCandidatePts.at(j).m_PtInd) = -1;
            }
        }
        else
        {
            vector<PtPolar> vecGroundPts;
            vecGroundPts.clear();
            vector<PtPolar> vecNoGroundPts;
            vecNoGroundPts.clear();
            // GPR
            GroundPtsEvaluator(vecCandidatePts, tmpvecPtsSection, vecGroundPts, vecNoGroundPts);
            for (int j = 0; j < vecGroundPts.size(); ++j)
            {
                m_vecbGround.at(vecGroundPts.at(j).m_PtInd) = 0;
            }
            for (int j = 0; j < vecNoGroundPts.size(); ++j)
            {
                m_vecbGround.at(vecNoGroundPts.at(j).m_PtInd) = 1;
            }
        }
    } // End of Section loop
}

void GroundSegmentor::SortByDistanceAcsending(vector<PtPolar> &vecPtsSection)
{
    for (int i = 1; i < vecPtsSection.size(); ++i)
    {
        for (int j = i; j > 0; --j)
        {
            if (vecPtsSection.at(j).m_Dis < vecPtsSection.at(j - 1).m_Dis)
            {
                PtPolar tmpPt = vecPtsSection.at(j);
                vecPtsSection.at(j) = vecPtsSection.at(j - 1);
                vecPtsSection.at(j - 1) = tmpPt;
            }
        }
    }
}

void GroundSegmentor::GroundPtsEvaluator(const vector<PtPolar> &vecCandidatePts, const vector<PtPolar> &vecTestingPts,
                                        vector<PtPolar> &vecGroundPts, vector<PtPolar> &vecNoGroundPts)
{
    vecGroundPts.clear();
    vecNoGroundPts.clear();

    // Trainng Data
    Eigen::VectorXf vTrainingX(vecCandidatePts.size());
    Eigen::VectorXf vTrainingY(vecCandidatePts.size());
    for (int i = 0; i < vecCandidatePts.size(); i++)
    {
        vTrainingX(i) = vecCandidatePts.at(i).m_Dis;
        vTrainingY(i) = vecCandidatePts.at(i).m_height;
    }

    // Predicting Data
    Eigen::VectorXf vTestingX(vecTestingPts.size());
    Eigen::VectorXf vTestingY(vecTestingPts.size());
    Eigen::VectorXf vTestingCov(vecTestingPts.size());
    for (int i = 0; i < vecTestingPts.size(); i++)
    {
        vTestingX(i) = vecTestingPts.at(i).m_Dis;
    }

    GPRPredictor(vTrainingX, vTrainingY, vTestingX, vTestingY, vTestingCov);

    for (int i = 0; i < vecTestingPts.size(); ++i)
    {
        double ratio;
        ratio = (vTestingY(i) - vecTestingPts.at(i).m_height) / sqrt(vTestingCov(i));
        if (abs(ratio) <= m_Param.m_GPParam.m_ThresholdCov)
        {
            vecGroundPts.push_back(vecTestingPts.at(i));
        }
        else
        {
            vecNoGroundPts.push_back(vecTestingPts.at(i));
        }
    }
}

void GroundSegmentor::GPRPredictor(const Eigen::VectorXf &vTrainingX, const Eigen::VectorXf &vTrainingY, 
                                    const Eigen::VectorXf &vPredictingX, 
                                    Eigen::VectorXf &vPredictingY, Eigen::VectorXf &vPredictingCov)
{
    double m0 = m_Param.m_GPParam.m_Mean.first;
    double m1 = m_Param.m_GPParam.m_Mean.second;
    double sn = exp(m_Param.m_GPParam.m_Lik);
    double sn2 = sn * sn;
    int n = vTrainingX.size();
    int m = vPredictingX.size();

    Eigen::MatrixXf Kuu(m, m);
    CalculateK(vPredictingX, vPredictingX, m_Param.m_GPParam.m_Cov, Kuu);

    Eigen::MatrixXf Kuf(m, n);
    CalculateK(vPredictingX, vTrainingX, m_Param.m_GPParam.m_Cov, Kuf);

    Eigen::MatrixXf Kff(n, n);
    CalculateK(vTrainingX, vTrainingX, m_Param.m_GPParam.m_Cov, Kff);

    Eigen::MatrixXf A = Eigen::MatrixXf::Identity(n, n);

    A = Kff + sn2 * A;
    Eigen::MatrixXf L = A.llt().matrixL();
    L = L.inverse();
    A = L.transpose() * L;

    Eigen::VectorXf V1 = Eigen::VectorXf::Constant(vPredictingX.size(), m1);
    Eigen::VectorXf V2 = Eigen::VectorXf::Constant(vTrainingX.size(), m1);

    vPredictingY = m0 * vPredictingX + V1 + Kuf * A * (vTrainingY - m0 * vTrainingX - V2);

    Eigen::MatrixXf Mfs2 = Kuu - Kuf * A * Kuf.transpose();
    Eigen::VectorXf fs2 = Mfs2.diagonal();
    Eigen::VectorXf V3 = Eigen::VectorXf::Constant(vPredictingY.size(), sn2);
    vPredictingCov = fs2 + V3;
}

void GroundSegmentor::CalculateK(const Eigen::VectorXf &x, const Eigen::VectorXf &y, const pair<double, double> &cov, Eigen::MatrixXf &K)
{
    double l2 = pow(exp(cov.first), 2);
    double sf2 = pow(exp(cov.second), 2);

    int n = x.size();
    int m = y.size();
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
        {
            K(i, j) = sf2 * exp(pow(x(i) - y(j), 2) * -0.5 / l2);
        }
    }
}

void GroundSegmentor::UpdateOutput()
{
    m_vecGroundInd.clear();
    m_vecNoGroundInd.clear();
    for (int i = 0; i < m_vecbGround.size(); ++i)
    {
        switch (m_vecbGround.at(i))
        {
        case 0:
            m_vecGroundInd.push_back(i);
            break;
        case 1:
            m_vecNoGroundInd.push_back(i);
            break;
        case -1:
            m_vecOutlierInd.push_back(i);
            break;
        }
    }
}