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
    std::cout<<"debug in constructor"<<std::endl;
    vector<double> tmp = {-0.0548588, 0.0358857, 1.94072, -0.24, 0.57};
    Eigen::Vector3d v3d(-0.0548588, 0.0358857, 1.94072);
    Sophus::SO3d so3d = Sophus::SO3d::exp(v3d);
    msCyRotation = so3d;
    std::cout<<"debug in constructor 1"<<std::endl;
    mdQx = tmp[3];
    mdCyr = tmp[4];
    std::cout<<"debug in constructor 2"<<std::endl;
    mnId=nNextId++;
    std::cout<<"debug in constructor 3"<<mnId<<std::endl;
}

Sophus::SO3d MapCylinder::GetSO3Para() {
    return msCyRotation;
}

double MapCylinder::GetCyr() {
    return mdCyr;
}

double MapCylinder::GetQx() {
    return mdQx;
}

Sophus::SE3f MapCylinder::GetTcyw() {
    return mTcyw;
}

//估计成功后修改pCy参数
void MapCylinder::SetWorldPara(Sophus::SO3d SO3, double qx, double r) {
    msCyRotation = SO3;
    mdQx = qx;
    mdCyr = r;
    this->SetWorldMat();
}

//通过自身的参数修改Matrix
void MapCylinder::SetWorldMat() {
    mRcyw = msCyRotation.matrix().cast<float>();
    mRwcy = mRcyw.transpose();
    Eigen::Vector3f t_cy((float) mdQx, 0., 0.);
    
    mPwcy = -mRwcy * t_cy;
    mPcyw = -mPwcy;
    Sophus::SE3f tmp(mRwcy, mPwcy);
    mTwcy = tmp;
    mTcyw = mTwcy.inverse();
    mDir = Eigen::Vector3f(mRwcy(0,2), mRwcy(1,2), mRwcy(2, 2));
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

//进行估计前准备初值 用PCA的方法判断圆柱初值 得到圆柱参数
 void MapCylinder::cyPreparation(vector<KeyFrame*> vpNeighKFs) {
    int sz = vpNeighKFs.size();
    cv::Mat data_pts = cv::Mat(sz, 3, CV_64FC1);
    //获取keyframes数据
    for(int i = 0; i < sz; ++i) {
        Eigen::Vector3f kfpose = vpNeighKFs[i]->GetPose().translation();
        data_pts.at<float>(i, 0) = kfpose[0];
        data_pts.at<float>(i, 1) = kfpose[1];
        data_pts.at<float>(i, 2) = kfpose[2];
    }
    std::cout<<"debug Mat data_pts"<<data_pts<<std::endl;
    cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
    //获取均值 关键帧位置(xyz)的中点 近似认为法线在该点P上
    cv::Point3f Point(pca_analysis.mean.at<float>(0,0), pca_analysis.mean.at<float>(0,1), pca_analysis.mean.at<float>(0,2));
    std::cout<<"Point: "<<Point<<std::endl;
    //获取轴向向量normal 获取特征值最大的法向量(就第一行)
    cv::Mat eigenVectors = pca_analysis.eigenvectors;
    cv::Point3f normal(eigenVectors.at<float>(0, 0), eigenVectors.at<float>(0, 1), eigenVectors.at<float>(0, 2));
    std::cout<<"normal: "<<normal<<std::endl;
    normal /= cv::norm(normal);
    
    
    //bug 不是这样的
    //设T点为世界坐标系原点向该直线的垂足，OP dot TP向量的值为 OP在向量上的投影TP的长度
    float cos_OP_PT = Point.dot(normal) / (cv::norm(Point) * cv::norm(normal));
    //TP
    float scale = cv::norm(Point) * cos_OP_PT;    //TP的长度
    cv::Point3f PT = -scale * normal;   //向量PT
    cv::Vec3f OT = static_cast<cv::Vec3f>(Point + PT);  //得到垂线段向量OT
    cv::Vec3f TO = -OT;
    float qx = cv::norm(OT);    //求出OT向量的模长 即圆柱参数qx
    std::cout<<"qx"<<qx<<std::endl;
    //对OT 和 normal归一化
    TO /= qx;
    
    //计算xyz轴旋转向量
    cv::Vec3f z_axis(normal.x, normal.y, normal.z);
    cv::Vec3f x_axis = TO;
    cv::Vec3f y_axis = TO.cross(normal); 
    y_axis /= cv::norm(y_axis);     //还用归一化一次吗
    Eigen::Vector3f x_axis_eigen;
    Eigen::Vector3f y_axis_eigen;
    Eigen::Vector3f z_axis_eigen;
    cv::cv2eigen(z_axis, z_axis_eigen);
    cv::cv2eigen(y_axis, y_axis_eigen);
    cv::cv2eigen(x_axis, x_axis_eigen);
    std::cout<<"x_axis_eigen"<<x_axis_eigen<<std::endl;
    std::cout<<"y_axis_eigen"<<x_axis_eigen<<std::endl;
    std::cout<<"z_axis_eigen"<<x_axis_eigen<<std::endl;
    Eigen::Matrix3f R;
    R<< x_axis_eigen, y_axis_eigen, z_axis_eigen; //Rwcy
    R = R.inverse();    //Rcyw
    std::cout<<"R: "<<R<<std::endl;
    Sophus::SO3d Rd(R.cast<double>());
    std::cout<<"debug cyPreparation"<<std::endl;
    this->msCyRotation = Rd;
    this->mdQx = static_cast<double>(qx);
    this->mdCyr = 0.5;
 }

 void MapCylinder::AddPastMapPoint(MapPoint* LandMark) {
    mlpPastMapPoints.push_back(LandMark);
 }

 void MapCylinder::AddCyMapPoint(MapPoint* LandMark) {
    mlpCyMapPoints.push_back(LandMark);
 }

void MapCylinder::AddCylindricalKF(KeyFrame* kf) {
    mlpCylindricalKF.emplace_back(kf);
}

void MapCylinder::CalculateLength() {

}

void MapCylinder::updateLength() {

}

}