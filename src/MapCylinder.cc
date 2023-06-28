//
// Created by Jankin on 17/06/2023.
//

#include "MapCylinder.h"

namespace ORB_SLAM3 {

long unsigned int MapCylinder::nNextId=0;

MapCylinder::MapCylinder(const vector<double> &Para, Map* pMap):mnBALocalForKF(0),mnLoopClosingForKF(0),mpMap(pMap),bActive(false),bBuilt(false),mfstart(0),mfend(0),mMaxMPonAxis(-100),mMinMPonAxis(100){
    msCyRotation = Sophus::SO3d::exp(Eigen::Vector3d(Para[0], Para[1], Para[2]));
    mdQx = Para[3];
    mdCyr = Para[4];
    //ToCyMatrix();
    mnId=nNextId++;
    //inverseVariance=pow(39.2/mPara.at<float>(4),2);
}

MapCylinder::MapCylinder(const vector<double> &Para):bActive(false),bBuilt(false),mfstart(0),mfend(0),mMaxMPonAxis(-100),mMinMPonAxis(100),mnBALocalForKF(0),mnLoopClosingForKF(0){
    msCyRotation = Sophus::SO3d::exp(Eigen::Vector3d(Para[0], Para[1], Para[2]));
    mdQx = Para[3];
    mdCyr = Para[4];
    //ToCyMatrix();
    mnId=nNextId++;
    //inverseVariance=pow(39.2/mPara.at<float>(4),2);
}

MapCylinder::MapCylinder(Map* pMap):mnBALocalForKF(0),mnLoopClosingForKF(0),mpMap(pMap),bActive(false),bBuilt(false),mfstart(0),mfend(0),mMaxMPonAxis(-100),mMinMPonAxis(100){
    vector<double> tmp = {-0.1, -0.1, 3.0, -0.24, 0.57};
    msCyRotation = Sophus::SO3d::exp(Eigen::Vector3d(tmp[0], tmp[1], tmp[2]));
    mdQx = tmp[3];
    mdCyr = tmp[4];
    mnId=nNextId++;
}

void MapCylinder::SetWorldPara(Sophus::SO3d Rotation, double qx, double r) {
    msCyRotation = Rotation;
    mdQx = qx;
    mdCyr = r;
}

//若该圆柱不准确，修改该圆柱存的地图点和关键帧指向的圆柱对象（删除）
void MapCylinder::SetBadFlag(){
    for(list<MapPoint*>::iterator lit=mlpCyMapPoints.begin(), lend=mlpCyMapPoints.end(); lit!=lend; lit++){
        (*lit)->mpMapCylinder=static_cast<MapCylinder*>(NULL);
    }
    for(list<KeyFrame*>::iterator lit=mlpCylindricalKF.begin() , lend=mlpCylindricalKF.end(); lit!=lend; lit++){
        (*lit)->AddMapCylinder(NULL);
    }
    mpMap->EraseMapCylinder(this);
}

//用未识别圆柱前几帧的关键帧位姿 在三角化那里找？
 void MapCylinder::cyPreparation(vector<KeyFrame*> vpNeighKFs) {
    int sz = vpNeighKFs.size();
    cv::Mat data_pts = cv::Mat(sz, 3, CV_64F);

 }

 void MapCylinder::AddPastMapPoint(MapPoint* LandMark) {
    mlpPastMapPoints.push_back(LandMark);
 }

 void MapCylinder::AddCyMapPoint(MapPoint* LandMark) {
    mlpCyMapPoints.push_back(LandMark);
 }

void MapCylinder::AddCylindricalKF() {
    
}

}