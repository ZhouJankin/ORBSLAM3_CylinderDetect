#ifndef ORB_SLAM2_MAPCYLINDER_H
#define ORB_SLAM2_MAPCYLINDER_H

#include "Map.h"
#include "Converter.h"
#include "MapPoint.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>
#include<vector>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM3
{
    class KeyFrame;
    class Map;
    class MapPoint;
    // class Frame;
    // class KeyFrame;

    class MapCylinder {
    public:
        MapCylinder(const vector<double> &Para, Map* pMap);
        MapCylinder(const vector<double> &Para);
        MapCylinder(Map *pMap);

        Sophus::SO3d GetSO3Para(); //得到其中哪个参数？写三个？
        double GetCyr();
        double GetQx();
        Sophus::SE3f GetTcyw();
        void SetWorldPara(Sophus::SO3d SO3, double qx, double r);  //设置参数
        void SetWorldMat(); //？
        void SetBadFlag();
        void cyPreparation(vector<KeyFrame*> NeighKFs);   //准备初值 用PCA的方法判断圆柱初值
        void AddPastMapPoint(MapPoint* LandMark); //估计前用该函数向mlpPastMapPoints加入地图点，当估计失败时，该函数删除一半过去地图点
        void AddCyMapPoint(MapPoint* LandMark);   //估计成功后，所有过去地图点的内点变为当前圆柱点，删除所有过去地图点
        void AddCylindricalKF(KeyFrame* kf);    //标记关键帧，证明圆柱信息的延续（当前关键帧估计出了圆柱）这个关键帧三角化出的地图点大部分都在圆柱里边
        void CalculateLength(); //计算起点终点
        void updateLength();    //更新起点终点，用于显示

    public:
        long unsigned int mnId; 
        static long unsigned int nNextId;
        long unsigned int mnBALocalForKF; //标记在哪个KF参与了LocalBA
        long unsigned int mnLoopClosingForKF;   //标记在哪个KF参与了LoopClosing
        list<MapPoint*> mlpPastMapPoints, mlpCyMapPoints; //储存地图点
        list<float> mlMPsOnAxis;  //用于计算圆柱长度(地图点在圆柱轴线上的投影)
        bool bActive;   //
        bool bBuilt;    //
        int mMaxMPonAxis;   //?
        int mMinMPonAxis;   //?

    protected:
        Sophus::SO3d msCyRotation; //Rcyw
        double mdQx;    //t_cy_w
        double mdCyr;
        //cv::Mat mPara;     //5 parameters, world coordinate
        Sophus::SE3f mTwcy;     //T metrix, world coordinate
        Eigen::Matrix3f mRwcy;      //Rotation matrix, world coordinate
        Eigen::Vector3f mPwcy;       //position, world coordinate
        Sophus::SE3f mTcyw;     //T metrix, world coordinate
        Eigen::Matrix3f mRcyw;      //Rotation matrix, world coordinate
        Eigen::Vector3f mPcyw;       //position, world coordinate
        Eigen::Vector3f mDir;           //direction, world coordinate 这是向量
        float mfstart, mfend; //标记起点，终点

        list<KeyFrame*> mlpCylindricalKF;
        std::map<KeyFrame*,size_t> mObservations;   

        Map* mpMap;


    };

}

#endif //ORB_SLAM2_MAPCYLINDER_H