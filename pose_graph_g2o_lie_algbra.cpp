#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <sophus/se3.hpp>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// Jacobian Matrix, J_r^{-1}
Matrix6d JRInv(const Sophus::SE3d &e) {
    Matrix6d J;
    J.block(0, 0, 3, 3) = Sophus::SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = Sophus::SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
    J.block(3, 3, 3, 3) = Sophus::SO3d::hat(e.so3().log());

    J = J * 0.5 + Matrix6d::Identity();
    // J = Matrix6d::Identity();
    return J;
}

// vertex
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class VertexSE3LieAlgebra : public g2o::BaseVertex<6, Sophus::SE3d> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual bool read(std::istream &is) override {
        double data[7];
        for (int i = 0; i < 7; ++i)
            is >> data[i];
        setEstimate(Sophus::SE3d(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]), // w, x, y, z
                Eigen::Vector3d(data[0], data[1], data[2])));
    }

    virtual bool write(std::ostream &os) const override {
        os << id() << " ";
        Eigen::Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " "
           << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;

        return true;
    }

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double *update) override {
        Vector6d upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(upd) * _estimate;
    }
};

class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual bool read(std::istream &is) override {
        double data[7];
        for (int i = 0; i < 7; ++i)
            is >> data[i];
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(Sophus::SE3d(q, Eigen::Vector3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); ++i)
            for (int j = i; j < information().cols() && is.good(); ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    virtual bool write(std::ostream &os) const override {
        VertexSE3LieAlgebra *v1 = static_cast<VertexSE3LieAlgebra *>(_vertices[0]);
        VertexSE3LieAlgebra *v2 = static_cast<VertexSE3LieAlgebra *>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        Sophus::SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                os << information()(i, j) << " ";
            }
        os << std::endl;
        return true;
    }

    virtual void computeError() override {
        Sophus::SE3d v1 = (static_cast<VertexSE3LieAlgebra *>(_vertices[0]))->estimate();
        Sophus::SE3d v2 = (static_cast<VertexSE3LieAlgebra *>(_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log(); // e_ij = ln(T_ij^{-1} * T_i^{-1} * T_j) (10.4)
    }

    virtual void linearizeOplus() override {
        Sophus::SE3d v1 = (static_cast<VertexSE3LieAlgebra *>(_vertices[0]))->estimate(); // T_i
        Sophus::SE3d v2 = (static_cast<VertexSE3LieAlgebra *>(_vertices[1]))->estimate(); // T_j
        Matrix6d J = JRInv(Sophus::SE3d::exp(_error));

        _jacobianOplusXi = -J * v2.inverse().Adj(); // v2 = T_j
        _jacobianOplusXj = J * v2.inverse().Adj();
    }
};

std::string file_name = "../data/sphere.g2o";

int main(int argc, char** argv) {

    std::ifstream fin(file_name);
    if (!fin) {
        std::cout << "file " << file_name << " does not exist." << std::endl;
        return 1;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int vertexCnt = 0, edgeCnt = 0;

    std::vector<VertexSE3LieAlgebra *> vertices;
    std::vector<EdgeSE3LieAlgebra *> edges;
    while (!fin.eof()) {
        std::string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            VertexSE3LieAlgebra *v = new VertexSE3LieAlgebra();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            vertices.push_back(v);
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();
            int idx1, idx2;
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }
        if (!fin.good()) break;
    }

    std::cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << std::endl;

    std::cout << "optimizing ..." << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    std::cout << "saving optimization results ..." << std::endl;

    std::ofstream fout("../results/retult_lie.g2o");
    for (VertexSE3LieAlgebra *v:vertices) {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }

    for (EdgeSE3LieAlgebra *e:edges) {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();
    return 0;
}