/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     batchG2oOptimizer.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:09:48
*/
#include "batchG2oOptimizer.hh"

BATCHOPTIMIZER_HH::BATCHOPTIMIZER_HH()
{
}

BATCHOPTIMIZER_HH::~BATCHOPTIMIZER_HH()
{

}

// 添加节点
bool BATCHOPTIMIZER_HH::AddVertex(g2o::VertexSE3 *v)
{
    g2o::VertexSE3* vertex(new g2o::VertexSE3);
    double est[7];

    v->getEstimateData(est);
    vertex->setEstimateData(est);
    vertex->setId(_v_id);
    vertex->setFixed(v->fixed());

    if(_optimizer.addVertex(vertex))
    {
        vertexMap[_v_id] = v;
        _v_id++;
        return true;
    }
    return false;
}

// 添加边
bool BATCHOPTIMIZER_HH::AddEdge(g2o::EdgeSE3 *e)
{
    if(_optimizer.addEdge(e))
        return true;
    return false;
}

void BATCHOPTIMIZER_HH::CreateVertexSE3(const number_t *estimate, g2o::VertexSE3 &v)
{
    v.setId(_v_id);
    v.setEstimateData(estimate);
//    v.scan_data = data;
}

void BATCHOPTIMIZER_HH::CreateEdgeSE3(g2o::VertexSE3 *vj, g2o::EdgeSE3 &e)
{
    e.vertices()[0] = _optimizer.vertex(vj->id()-1);
    e.vertices()[1] = _optimizer.vertex(vj->id());
    e.setMeasurementFromState();
    e.setInformation(g2o::EdgeSE3::InformationType::Identity());
}

void BATCHOPTIMIZER_HH::CreateEdgeSE3(g2o::VertexSE3 *vi, g2o::VertexSE3 *vj, number_t *d, g2o::EdgeSE3 &e)
{
    e.vertices()[0] = _optimizer.vertex(vi->id());
    e.vertices()[1] = _optimizer.vertex(vj->id());
    e.setMeasurementData(d);
    e.setInformation(g2o::EdgeSE3::InformationType::Identity());
}

void BATCHOPTIMIZER_HH::OptimizeGraph(int iterations, bool initialize)
{
    if(initialize)
    {
        if(_optimizer.initializeOptimization())
        {
            std::cout << "Start Otimization ....." << std::endl;
            _optimizer.optimize(iterations);
            std::cout << "Graph Otimization Done" << std::endl;
        }
    }
    else
    {
        std::cout << "Start Otimization ....." << std::endl;
        _optimizer.optimize(iterations);
        std::cout << "Graph Otimization Done" << std::endl;
    }

    for(int i=0; i<_optimizer.vertices().size(); i++)
    {
        double est[7];
        _optimizer.vertex(i)->getEstimateData(est);
        vertexMap[i]->setEstimateData(est);
    }

}

void BATCHOPTIMIZER_HH::RPYposeToMeasurement(const RPYpose pose, number_t *d)
{
    XSTF::Quaternion q;
    q.setRPY(pose.roll, pose.pitch, pose.yaw);

    d[0] = pose.x;
    d[1] = pose.y;
    d[2] = pose.z;
    d[3] = q.x();
    d[4] = q.y();
    d[5] = q.z();
    d[6] = q.w();
}

void BATCHOPTIMIZER_HH::MatrixToMeasurement(const Eigen::Matrix4f matrix, number_t *d)
{
    XSTF::Matrix3x3 mat33;

    mat33.setValue(static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)),
                   static_cast<double>(matrix(0, 2)), static_cast<double>(matrix(1, 0)),
                   static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
                   static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)),
                   static_cast<double>(matrix(2, 2)));

    XSTF::Quaternion q;
    mat33.getRotation(q);

    // Update measurement data
    d[0] = matrix(0, 3);
    d[1] = matrix(1, 3);
    d[2] = matrix(2, 3);
    d[3] = q.x();
    d[4] = q.y();
    d[5] = q.z();
    d[6] = q.w();

}

void BATCHOPTIMIZER_HH::EstimateToMatrix(const number_t *d, Eigen::Matrix4f& matrix)
{
    Eigen::Translation3f translation(d[0],d[1],d[2]);
    Eigen::Quaternionf q;
    q.x() = d[3];
    q.y() = d[4];
    q.z() = d[5];
    q.w() = d[6];
    Eigen::Matrix3f rotation = q.normalized().toRotationMatrix();
    matrix = (translation*rotation).matrix();
}

void BATCHOPTIMIZER_HH::EstimateToRPYpose(const number_t *d, RPYpose& pose)
{
    pose.x = d[0];
    pose.y = d[1];
    pose.z = d[2];

    XSTF::Quaternion q;
    q.setValue(d[3], d[4], d[5], d[6]);
    XSTF::Matrix3x3(q).getRPY(pose.roll, pose.pitch, pose.yaw);
}

void BATCHOPTIMIZER_HH::AddLcEge(const int input_id, const int target_id, double *d)
{
    std::cout << "add edge " << input_id << " " << target_id << std::endl;
    g2o::EdgeSE3 *LCedge(new g2o::EdgeSE3);
    CreateEdgeSE3(vertexMap[target_id],vertexMap[input_id],d,*LCedge);
    AddEdge(LCedge);
//    SLAM3DSystem::ids ids;
//    ids.i = target_id;
//    ids.j = input_id;
//    _LC_id_map.push_back(ids);
}
