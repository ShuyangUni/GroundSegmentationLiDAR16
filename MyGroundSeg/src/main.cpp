#include "BasicSource.h"
#include "GroundSegmentor.h"
#include <pcl/visualization/cloud_viewer.h>

int main(int, char **)
{
    // Load PCD Data
    string filename = "/home/uni/DiskData/PCD/white_car_top/1546671800.174543000.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
    {
        cout << "LoadPCDFile is wrong!" << endl;
        return -1;
    }

    // Some Info : rslidar_16 - Height : 16, Width : 2040
    // LaserNumberOrder: 1-8,16-9
    // reRank Data
    //
    // for (int i = 8; i < 12; i++)
    // {
    //     for (int j = 0; j < 2040; j++)
    //     {
    //         int idx1 = i * 2040 + j;
    //         int idx2 = (23 - i) * 2040 + j;
    //         pcl::PointXYZ tPt = cloud->points[idx1];
    //         cloud->points[idx1] = cloud->points[idx2];
    //         cloud->points[idx2] = tPt;
    //     }
    // }

    clock_t start_time=clock();
    // Ground Segmentation
    Param myParam;
    GroundSegmentor myGS(cloud, myParam);
    myGS.run();

    vector<int> vecGroundInd;
    vector<int> vecNoGroundInd;
    vector<int> vecOutlierInd;
    myGS.output(vecGroundInd, vecNoGroundInd, vecOutlierInd);

    clock_t end_time=clock();
    cout << "The run time is: " <<(double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;

    cout <<"ObjectPtsNum : "<< vecNoGroundInd.size() << " "
         <<"GroundPtsNum : "<< vecGroundInd.size() << " "
         <<"OutlierPtsNum : "<< vecOutlierInd.size() << endl;

    // Display
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDisplay(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < vecNoGroundInd.size(); ++i)
    {
        pcl::PointXYZRGB tPtXYZRGB;
        pcl::PointXYZ tPtXYZ = cloud->points[vecNoGroundInd.at(i)];
        tPtXYZRGB.x = tPtXYZ.x;
        tPtXYZRGB.y = tPtXYZ.y;
        tPtXYZRGB.z = tPtXYZ.z;
        tPtXYZRGB.r = 0;
        tPtXYZRGB.g = 0;
        tPtXYZRGB.b = 255;
        cloudDisplay->points.push_back(tPtXYZRGB);
    }
    
    for (int i = 0; i < vecGroundInd.size(); ++i)
    {
        pcl::PointXYZRGB tPtXYZRGB;
        pcl::PointXYZ tPtXYZ = cloud->points[vecGroundInd.at(i)];
        tPtXYZRGB.x = tPtXYZ.x;
        tPtXYZRGB.y = tPtXYZ.y;
        tPtXYZRGB.z = tPtXYZ.z;
        tPtXYZRGB.r = 255;
        tPtXYZRGB.g = 0;
        tPtXYZRGB.b = 0;
        cloudDisplay->points.push_back(tPtXYZRGB);
    }

    pcl::visualization::CloudViewer viewer ("Cluster viewer");  
    viewer.showCloud(cloudDisplay);
    while (!viewer.wasStopped ())  
    {
        sleep(1);
    }       
    return 0;
}