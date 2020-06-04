#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

std::string file_name = "../data/sphere.g2o";

int main(int argc, char** argv) {
    std::ifstream fin(file_name);
    if (!fin) {
        std::cout << "file " << file_name << "does not exist." << std::endl;
        return 1;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType; // information matrix
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int vertexCnt = 0, edgeCnt = 0; // number of vertex and edge
    while (!fin.eof()) {
        std::string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            // g2o::VertexSE3: pose (Quaternion, t)
            // ID, tx, ty, tz, qx, qy, qz, qw
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index; // ID
            v->setId(index);
            v->read(fin); // t_xyz, q_xyz
            optimizer.addVertex(v);
            vertexCnt++;
            if (index == 0)
                v->setFixed(true);

        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 edge
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2; // two vertex connected by edge
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]); // set connected vertex
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if (!fin.good()) break;
    }

    std::cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << std::endl;

    std::cout << "optimizing ..." << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    std::cout << "saving optimization results ..." << std::endl;
    optimizer.save("../results/result.g2o");

    return 0;
}
