/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <math.h>
#include <vector>
#include <set>
#include<list>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{

class LoopClosing;

class Optimizer
{
public:

    class CylinderIntrinsics {
    public:
    CylinderIntrinsics() {}

    //从给的Vector5d中得到参数放入结构
    CylinderIntrinsics(vector<double> data_addr) {
        rotation = Sophus::SO3d::exp(Eigen::Vector3d(data_addr[0], data_addr[1], data_addr[2]));
        qx = data_addr[3];
        r = data_addr[4];
    }
    //返回估计值
    // Eigen::Matrix<double,5,1> estimate() {
    //     Eigen::Matrix<double, 5, 1> data_addr;
    //     auto so3 = rotation.log();
    //     for(int i = 0; i < 3; ++i) data_addr[i] = so3[i];
    //     data_addr[3] = qx;
    //     data_addr[4] = r;
    //     return data_addr;
    // }
    Sophus::SO3d rotation;
    double qx;
    double r;
};

    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges);

    int static PoseOptimization(Frame* pFrame);
    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);
    void static OptimizeEssentialGraph(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                       vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs);

    // For inertial loopclosing
    void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);


    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);

    // For inertial systems

    void static LocalInertialBA(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge = false, bool bRecInit = false);
    void static MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses);

    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);
    static std::vector<double> computeDistance(const std::vector<Eigen::Vector3d> &noisyLandmarks, CylinderIntrinsics abc);
    static double computeMean(const std::vector<double> &dists, const std::vector<bool> &isOuter, int InnerNum);
    static double computeStdDev(const std::vector<double> &dists, double mean, const std::vector<bool> &isOuter, int InnerNum);
    void static OptimizeOnce(std::vector<Eigen::Vector3d> &noisyLandmarks, std::vector<bool> OuterIndex, CylinderIntrinsics &abc);
    static bool LoopOptimization(std::vector<Eigen::Vector3d> &noisyLandmarks, CylinderIntrinsics &abc);

    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);

    // Inertial pose-graph
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);


    static CylinderIntrinsics mCyVar;
    static bool mbCyVar;
    static std::set<MapPoint*> msUsedPoints;
    static std::list<MapPoint*> mlLastLocalBAPoints;
    static std::vector<Eigen::Vector3d> mvLandMarks;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class CylinderFittingVertex : public g2o::BaseVertex<5, Optimizer::CylinderIntrinsics> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置
    virtual void setToOriginImpl() override {
        _estimate = Optimizer::CylinderIntrinsics();
    }

    // 更新
    virtual void oplusImpl(const double *update) override {
        _estimate.rotation = Sophus::SO3d::exp(Eigen::Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.qx += update[3];
        _estimate.r += update[4];

        // Eigen::Matrix<double,5,1> last=_estimate;
        // //先更新qx和r
        // _estimate.block(3, 0, 2, 1) += Eigen::Matrix<double,5,1>(update).block(3, 0, 2, 1);
        // Eigen::Vector3d so3 ;
        // so3 << update[0], update[1], update[2];
        // _estimate.block(0, 0, 3, 1) = Sophus::SO3d::exp(so3) * _estimate.block(0, 0, 3, 1);
    }

    // 存盘和读盘：留空
    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}
};


class CylinderFittingEdge : public g2o::BaseBinaryEdge<1, double, g2o::VertexSBAPointXYZ, CylinderFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 计算曲线模型误差
    virtual void computeError() override {
        const g2o::VertexSBAPointXYZ *vPoint = static_cast<const g2o::VertexSBAPointXYZ *> (_vertices[0]);
        const CylinderFittingVertex *v = static_cast<const CylinderFittingVertex *> (_vertices[1]);
        const Optimizer::CylinderIntrinsics abc = v->estimate();
        const Eigen::Vector3d p = vPoint->estimate();
        Eigen::Matrix3d A;
        A << 1., 0., 0., 0., 1., 0., 0., 0., 0.;
        Eigen::Vector3d u = A *((abc.rotation.matrix() * p) + Eigen::Vector3d(abc.qx, 0, 0));
        double norm = u.transpose() * u;

        //todo 什么时候误差写作_error(0,0)什么时候写作_error ??
        //update 将error 变为距离轴线的距离+ abc.r - 0.5 来保证圆柱半径跟0.5接近
        _error(0, 0) = sqrt(norm) - abc.r;

    }

    // 计算雅可比矩阵
    virtual void linearizeOplus() override {
        const g2o::VertexSBAPointXYZ *vPoint = static_cast<const g2o::VertexSBAPointXYZ *> (_vertices[0]);
        const CylinderFittingVertex *v = static_cast<const CylinderFittingVertex *> (_vertices[1]);
        const Optimizer::CylinderIntrinsics abc = v->estimate();
        const Eigen::Vector3d p = vPoint->estimate();

        //先计算一些中间变量
        Eigen::Matrix3d A;
        A << 1., 0., 0., 0., 1., 0., 0., 0., 0.;
        Eigen::Vector3d Rp = abc.rotation.matrix() * p;
        Eigen::Vector3d qx(abc.qx, 0., 0.);
        double u = (A *(Rp + qx)).transpose() * (A * (Rp+ qx));
        // double u = Rp.transpose() * A.transpose() * A * Rp;
        // u += Eigen::Vector3d (abc.qx, 0., 0.).transpose() * A.transpose() * A * Rp;
        // //todo ? * 2.0
        // u +=  (Rp.transpose() * A.transpose() * A * Eigen::Vector3d(abc.qx, 0., 0.))[0] * 2.0;
        // u += Eigen::Vector3d (abc.qx, 0., 0.).transpose() * A.transpose() * A * Eigen::Vector3d (abc.qx, 0., 0.);
        //Rp的反对称矩阵
        Eigen::Matrix3d Rp_trans_inv;
        Rp_trans_inv << 0., -Rp(2), Rp(1), 
                                            Rp(2), 0., -Rp(0), 
                                            -Rp(1), Rp(0), 0.;

        //error对路标点p求偏导
        _jacobianOplusXi = (pow(u, -1./2)) * ((abc.rotation.matrix().transpose() * A * abc.rotation.matrix() * p).transpose() + qx.transpose() * A * abc.rotation.matrix());
       // std::cout<<_jacobianOplusXi<<std::endl;

        //error对圆柱参数求偏导
        // Eigen::Vector3d Rotation_def = (1./2 * pow(y, -1./2)) * (2.0 * (A * (Ryw * pw) + qx).transpose()) * (-Rp_hat);
        Eigen::Matrix<double, 1, 3> Jacobian_se3 = (1./2 * pow(u, -1./2)) * (2.0 * (A * Rp + qx).transpose()) * (-Rp_trans_inv);
        _jacobianOplusXj[0] = Jacobian_se3(0);
        _jacobianOplusXj[1] = Jacobian_se3(1);
        _jacobianOplusXj[2] = Jacobian_se3(2);
        _jacobianOplusXj[3] = (1./sqrt(u)) * (Eigen::Vector3d(1., 0., 0.).transpose() * A * Rp + abc.qx);
        _jacobianOplusXj[4] = -1.0;

    }

    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}
};

} //namespace ORB_SLAM3

#endif // OPTIMIZER_H
