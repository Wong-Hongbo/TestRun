/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     batchG2oOptimizer.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-11-14 11:29:16
*/
#ifndef BATCH_G2O_OPTIMIZER_HH_
#define BATCH_G20_OPTIMIZER_HH_
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/vertex_se3_euler.h>

#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/csparse/csparse_extension.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <g2o/stuff/command_args.h>
#include "Common/ParamServer.hh"

class BATCHOPTIMIZER_HH
{
public:
    BATCHOPTIMIZER_HH();

    ~BATCHOPTIMIZER_HH();

    // 添加节点
    bool AddVertex(g2o::VertexSE3 *v);

    // 添加边
    bool AddEdge(g2o::EdgeSE3 *e);
    void AddLcEge(const int input_id, const int target_id, double *d);
    void CreateVertexSE3(const number_t *estimate, g2o::VertexSE3 &v);
    void CreateEdgeSE3(g2o::VertexSE3 *vj, g2o::EdgeSE3 &e);
    void CreateEdgeSE3(g2o::VertexSE3 *vi, g2o::VertexSE3 *vj, number_t *d, g2o::EdgeSE3 &e);

    void OptimizeGraph(int iterations, bool initialize);
    void RPYposeToMeasurement(const RPYpose pose, number_t *d);
    void MatrixToMeasurement(const Eigen::Matrix4f matrix, number_t *d);
    void EstimateToMatrix(const number_t *d, Eigen::Matrix4f& matrix);
    void EstimateToRPYpose(const number_t *d, RPYpose& pose);

    void LoadData();

    double _LC_measurement[7]; // 内部四元数
    std::map<int, g2o::VertexSE3*> vertexMap;
    int _v_id;

private:
  g2o::SparseOptimizer _optimizer;
};


#endif//ISAM_OPTIMIZER_HH_
