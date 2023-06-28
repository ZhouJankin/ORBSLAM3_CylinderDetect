#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_sba.h>
#include"sophus/se3.hpp"


#ifndef VO_SIMULATE_CYLINDER_H
#define VO_SIMULATE_CYLINDER_H
#define pi 3.1415926535

struct CylinderIntrinsics {
    CylinderIntrinsics() {}

    //从给的Vector5d中得到参数放入结构
    explicit CylinderIntrinsics(double *data_addr) {
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



class CylinderFittingVertex : public g2o::BaseVertex<5, CylinderIntrinsics> {
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW


// 重置
virtual void setToOriginImpl() override {
    _estimate = CylinderIntrinsics();
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


class CylinderFittingEdge : public g2o::BaseBinaryEdge<1, double, g2o::VertexPointXYZ, CylinderFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 计算曲线模型误差
    virtual void computeError() override {
        const g2o::VertexPointXYZ *vPoint = static_cast<const g2o::VertexPointXYZ *> (_vertices[0]);
        const CylinderFittingVertex *v = static_cast<const CylinderFittingVertex *> (_vertices[1]);
        const CylinderIntrinsics abc = v->estimate();
        const Eigen::Vector3d p = vPoint->estimate();
        Eigen::Matrix3d A;
        A << 1., 0., 0., 0., 1., 0., 0., 0., 0.;
        Eigen::Vector3d u = A *((abc.rotation.matrix() * p) + Eigen::Vector3d(abc.qx, 0, 0));
        double norm = u.transpose() * u;

        //todo 什么时候误差写作_error(0,0)什么时候写作_error ??
        _error(0, 0) = sqrt(norm) - abc.r;

    }

    // 计算雅可比矩阵
    virtual void linearizeOplus() override {
        const g2o::VertexPointXYZ *vPoint = static_cast<const g2o::VertexPointXYZ *> (_vertices[0]);
        const CylinderFittingVertex *v = static_cast<const CylinderFittingVertex *> (_vertices[1]);
        const CylinderIntrinsics abc = v->estimate();
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


#endif //VO_SIMULATE_CYLINDER_H

