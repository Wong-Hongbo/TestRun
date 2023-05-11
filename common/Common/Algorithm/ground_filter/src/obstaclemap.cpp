//#######################################################################################
//#  This source code is about a ground filtering algorithm for airborn LiDAR data      #
//#  based on physical process simulations, specifically cloth simulation.              #
//#                                     Copyright 2021                                  #
//#                  Changsha Xingshen Intelligent Technology Co., Ltd                  #
//#                               (http://www.xingshentech.com/)                        #
//#                                                                                     #
//#                                  Authored By Meng Deyuan                            #
//#                                                                                     #
//#######################################################################################

#include "obstaclemap.h"


ObstacleMap::ObstacleMap(CarModelParams car_params, PolarGridmapParams polar_params):
    m_kCarModelParams(car_params),
    m_kPolarGridmapParams(polar_params)
{
    InitPolarGridmapParams();
    for(int i=0; i<ATAN_LIST_NUM; ++i)
    {
        int angle = int(std::atan(double(i) / 100.0) * RADIAN2ANGLE);
        m_atan_angle.push_back(angle);
    }
}


void ObstacleMap::InitPolarGridmapParams()
{
    // 1. 计算径向和角向的最大值，角度范围为：[angle_min, angle_max)
    m_radis_max_idx = int(std::round(m_kPolarGridmapParams.radis_range / m_kPolarGridmapParams.radis_resolution));
    int idx_min = int(std::floor(float(m_kPolarGridmapParams.angle_range_min) / m_kPolarGridmapParams.angle_resolution));
    int idx_max = int(std::ceil(float(m_kPolarGridmapParams.angle_range_max) / m_kPolarGridmapParams.angle_resolution));
    m_angle_max_idx = idx_max - idx_min;
    // 2. 计算径向栅格的中心位置
    m_radis_position.resize(m_radis_max_idx);
    for(int i=0; i<m_radis_max_idx; ++i)
    {
        m_radis_position[i] = (float(i) + 0.5) * m_kPolarGridmapParams.radis_resolution;
    }
    // 3. 计算每个水平角片区的盲区区域，盲区区域是指由于车身占据位置导致的径向不可见盲区
    int round_idx = int(std::round(360.0f / m_kPolarGridmapParams.angle_resolution));
    std::vector<int> blind_area(round_idx);
    int angle1;
    int angle2;
    float k;
    float r;
    // 3.1 后盲区边界，即左后角点和右后角点之间的盲区的极坐标表示
    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 180.0));
    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 540.0));
    for(int i=angle1; i<=angle2; ++i)
    {
        k = std::tan((-i + 270) * ANGLE2RADIAN);
        r = std::fabs(m_kCarModelParams.boundary_rear) * std::sqrt(1 / std::pow(k, 2) + 1);
        int search_idx = i % 360;
        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
    }
    // 3.2 左盲区边界，即左后角点和左前角点之间的盲区的极坐标表示
    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 180.0));
    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
    for(int i=angle1; i<=angle2; ++i)
    {
        k = std::tan((-i + 270) * ANGLE2RADIAN);
        r = std::fabs(m_kCarModelParams.boundary_left) * std::sqrt(std::pow(k, 2) + 1);
        int search_idx = i % 360;
        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
    }
    // 3.3 前盲区边界，即左前角点和右前角点之间的盲区的极坐标表示
    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
    for(int i=angle1; i<=angle2; ++i)
    {
        k = std::tan((-i + 270) * ANGLE2RADIAN);
        r = std::fabs(m_kCarModelParams.boundary_front) * std::sqrt(1 / std::pow(k, 2) + 1);
        int search_idx = i % 360;
        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
    }
    // 3.4 右盲区边界，即右后角点和右前角点之间的盲区的极坐标表示
    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 180.0));
    for(int i=angle1; i<=angle2; ++i)
    {
        k = std::tan((-i + 270) * ANGLE2RADIAN);
        r = std::fabs(m_kCarModelParams.boundary_right) * std::sqrt(std::pow(k, 2) + 1);
        int search_idx = i % 360;
        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
    }
    // 3.5 约束最小盲区为1
    for(uint i=0; i<blind_area.size(); ++i)
    {
        if(blind_area[i] < 2)
        {
            blind_area[i] = 2;
        }
        else if(blind_area[i] >= m_radis_max_idx - 1)
        {
            blind_area[i] = m_blind_area[i] - 1;
        }
    }
    for(int i=idx_min; i<idx_max; ++i)
    {
        int ii = i % blind_area.size();
        m_blind_area.push_back(blind_area[ii] - 1);
//        m_blind_area.push_back(int(std::ceil(float(blind_area[ii]) / 2)));
    }
    // 4. 初始化模拟地形
    // 4.1 创建地形
    m_cloth_terrain.resize(m_angle_max_idx);
    for(int i=0; i<m_angle_max_idx; ++i)
    {
        m_cloth_terrain[i].resize(m_radis_max_idx);
    }

    for(int i=0; i<m_radis_max_idx; ++i)
    {
        for(int j=0; j<m_angle_max_idx; ++j)
        {
            float angle = (j + idx_min + 0.5) * m_kPolarGridmapParams.angle_resolution * ANGLE2RADIAN;
            float pos_x = -std::sin(angle) * m_radis_position[i];
            float pos_y = -std::cos(angle) * m_radis_position[i];
            m_cloth_terrain[j][i].SetPosition(pos_x, pos_y);
            m_cloth_terrain[j][i].SetNeighborNum(25);
        }
    }

    bool round_flag = false;
    if(m_kPolarGridmapParams.angle_range_min == 0 && m_kPolarGridmapParams.angle_range_max == 360)
    {
        round_flag = true;
    }
    for(int i=0; i<m_radis_max_idx; ++i)
    {
        for(int j=0; j<m_angle_max_idx; ++j)
        {
            AddNeighbor(j, i, j, i-1, round_flag);
            AddNeighbor(j, i, j, i+1, round_flag);
            AddNeighbor(j, i, j-1, i, round_flag);
            AddNeighbor(j, i, j+1, i, round_flag);
            AddNeighbor(j, i, j-1, i-1, round_flag);
            AddNeighbor(j, i, j+1, i-1, round_flag);
            AddNeighbor(j, i, j-1, i+1, round_flag);
            AddNeighbor(j, i, j+1, i+1, round_flag);
            AddNeighbor(j, i, j, i-2, round_flag);
            AddNeighbor(j, i, j-1, i-2, round_flag);
            AddNeighbor(j, i, j+1, i-2, round_flag);
            AddNeighbor(j, i, j, i+2, round_flag);
            AddNeighbor(j, i, j-1, i+2, round_flag);
            AddNeighbor(j, i, j+1, i+2, round_flag);
        }
    }
}


void ObstacleMap::AddNeighbor(int curr_a, int curr_r, int neighbor_a, int neighbor_r, bool round_flag)
{
    if(neighbor_r >= 0 && neighbor_r < m_radis_max_idx)
    {
        if(round_flag)
        {
            int idx_a = (neighbor_a + m_angle_max_idx) % m_angle_max_idx;
            m_cloth_terrain[curr_a][curr_r].AddNeighbor(&m_cloth_terrain[idx_a][neighbor_r]);
        }
        else
        {
            if(neighbor_a >= 0 && neighbor_a < m_angle_max_idx)
            {
                m_cloth_terrain[curr_a][curr_r].AddNeighbor(&m_cloth_terrain[neighbor_a][neighbor_r]);
            }
        }
    }
}


int ObstacleMap::CountAtanAngle(float x, float y)
{
    int angle;
    int idx;
    float tan_v;
    if(y == 0)
    {
        if(x >= 0)
        {
            angle = 270;
        }
        else
        {
            angle = 90;
        }
    }
    else if(y > 0)
    {
        tan_v = 100 * x / y;
        if(tan_v >= 0)
        {
            idx = int(tan_v);
            angle = GetAtanAngleFromList(idx) + 180;

        }
        else
        {
            idx = int(-tan_v);
            angle = -GetAtanAngleFromList(idx) + 180;
        }
    }
    else
    {
        tan_v = 100 * x / y;
        if(tan_v >= 0)
        {
            idx = int(tan_v);
            angle = GetAtanAngleFromList(idx);

        }
        else
        {
            idx = int(-tan_v);
            angle = -GetAtanAngleFromList(idx) + 360;
            angle = angle % 360;
        }
    }

    return angle;
}


int ObstacleMap::GetAtanAngleFromList(int x)
{
    int angle;
    if(x >= ATAN_LIST_NUM)
    {
        angle = 90;
    }
    else
    {
        angle = m_atan_angle[x];
    }

    return angle;
}

void ObstacleMap::PointCloudPreprocess(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_data)
{
    m_valid_num = 0;
    m_pointclouds.clear();
    m_pointclouds.resize(lidar_data->size());
    // 迪卡尔坐标系转极坐标
    float min_z = MAX_INF;
    // 最大允许高度为限制高度的2倍
    float max_z = m_kCarModelParams.suspended_height * 2;
    for(int i=0; i<lidar_data->size(); ++i)
    {
        float x = lidar_data->points[i].x;
        float y = lidar_data->points[i].y;
        float z = lidar_data->points[i].z + m_kCarModelParams.ground_height;
        float intensity = lidar_data->points[i].intensity;
        if(x < m_kCarModelParams.boundary_left || x > m_kCarModelParams.boundary_right || \
           y < m_kCarModelParams.boundary_rear || y > m_kCarModelParams.boundary_front)
        {
            int angle = CountAtanAngle(x, y) - m_kPolarGridmapParams.angle_range_min;
            if(angle < 0)
            {
                angle = angle + 360;
            }
            m_pointclouds[m_valid_num].x = x;
            m_pointclouds[m_valid_num].y = y;
            m_pointclouds[m_valid_num].z = z;
            m_pointclouds[m_valid_num].intensity = intensity;
            float distance = std::sqrt(std::pow(x,2) + std::pow(y,2));
            m_pointclouds[m_valid_num].pos_a = angle / m_kPolarGridmapParams.angle_resolution;
            m_pointclouds[m_valid_num].pos_r = int(distance / m_kPolarGridmapParams.radis_resolution);
            m_pointclouds[m_valid_num].filter_a = angle;
            m_pointclouds[m_valid_num].filter_r = int(distance / FILTER_RES);
            m_pointclouds[m_valid_num].filter_z = int((m_pointclouds[m_valid_num].z - MINZ) / FILTER_RES);
            if(m_pointclouds[m_valid_num].pos_a >= m_angle_max_idx ||
               m_pointclouds[m_valid_num].pos_r >= m_radis_max_idx ||
               m_pointclouds[m_valid_num].z < MINZ ||
               m_pointclouds[m_valid_num].z >= max_z)
            {
                m_pointclouds[m_valid_num].type = 3;
            }
            else
            {
                // 记录有效点的最低点高度
                if(z < min_z)
                {
                    min_z = z;
                }
            }
            m_valid_num++;
        }
    }
    RasterTerrain(-min_z + CLOTH_Z_COMP);
}


void ObstacleMap::RasterTerrain(float max_h)
{
    // 织物地形图初始化
    float time_step2 = m_kPolarGridmapParams.time_step * m_kPolarGridmapParams.time_step;
    for(int i=0; i<m_radis_max_idx; ++i)
    {
        for(int j=0; j<m_angle_max_idx; ++j)
        {
            m_cloth_terrain[j][i].Init(max_h, time_step2);
        }
    }
    // 有效点云向织物地形图投影，获取地面最小高度
    cv::Mat occ_map = cv::Mat::zeros(m_angle_max_idx, m_radis_max_idx, CV_32S);
    cv::Mat height_map = cv::Mat::zeros(m_angle_max_idx, m_radis_max_idx, CV_32F);

    for(int i=0; i<m_valid_num; ++i)
    {
        if(m_pointclouds[i].type < 3)
        {
            int idx_r = m_pointclouds[i].pos_r;
            int idx_a = m_pointclouds[i].pos_a;
            float height = m_pointclouds[i].z;
            if(occ_map.at<int>(idx_a,idx_r) == 0)
            {
                height_map.at<float>(idx_a,idx_r) = height;
                occ_map.at<int>(idx_a,idx_r) = 1;
            }
            else
            {
                if(height_map.at<float>(idx_a,idx_r) > height)
                {
                    height_map.at<float>(idx_a,idx_r) = height;
                }
            }
        }
    }
    // 在占据的地形图上，将存在障碍的栅格标记为2用于后续处理
    // 存在障碍的定义是有点距离最低点大于障碍阈值
    float grad_thresh = std::tan(m_kPolarGridmapParams.slope_thresh * ANGLE2RADIAN);
    float min_bound = std::max(m_kPolarGridmapParams.height_thresh, grad_thresh * m_kPolarGridmapParams.radis_resolution);
    for(int i=0; i<m_valid_num; ++i)
    {
        if(m_pointclouds[i].type < 3)
        {
            int idx_r = m_pointclouds[i].pos_r;
            int idx_a = m_pointclouds[i].pos_a;
            float height = m_pointclouds[i].z;
            if(occ_map.at<int>(idx_a,idx_r) == 1)
            {
                float diff_h = height - height_map.at<float>(idx_a,idx_r);
                if(diff_h > min_bound && diff_h < m_kCarModelParams.suspended_height)
                {
                    occ_map.at<int>(idx_a,idx_r) = 2;
                }
            }
        }
    }


    // 通过径向搜索删除异常点
    for(int i=0; i<m_angle_max_idx; ++i)
    {
        // 盲区置占据，从非盲区开始搜索
        for(int curr_idx=0; curr_idx<=m_blind_area[i]; ++curr_idx)
        {
            occ_map.at<int>(i,curr_idx) = 1;
        }
        // 非盲区开始搜索
        int last_idx = m_blind_area[i];
        float last_z = 0;
        float last_r = m_radis_position[last_idx];
        int comp_idx = m_blind_area[i];
        float comp_z = 0;
        float comp_r = m_radis_position[comp_idx];
        for(int curr_idx=m_blind_area[i]+1; curr_idx<m_radis_max_idx; ++curr_idx)
        {
            // 如果点差距大于阈值则不再扩展退出本循环
            if(m_radis_position[curr_idx] - m_radis_position[comp_idx] > m_kPolarGridmapParams.max_search_dis)
            {
                for(int jj=curr_idx; jj<m_radis_max_idx; ++jj)
                {
                    occ_map.at<int>(i,jj) = 0;
                }
                break;
            }
            else
            {
                // 如果该点为地面占据点则判断梯度是否小于阈值，小于阈值扩展，否则删除
                float curr_r = m_radis_position[curr_idx];
                if(occ_map.at<int>(i,curr_idx) > 0)
                {
                    float curr_z = height_map.at<float>(i,curr_idx);
                    float slope = (curr_z - comp_z) / (curr_r - comp_r);
                    if(std::fabs(slope) < grad_thresh)
                    {
                        last_r = curr_r;
                        last_z = curr_z;
                        last_idx = curr_idx;
                        comp_r = curr_r;
                        comp_z = curr_z;
                        comp_idx = curr_idx;
                    }
                    else
                    {
                        comp_r = curr_r;
                        comp_idx = curr_idx;
                        occ_map.at<int>(i,curr_idx) = 0;
                    }
                }
            }
        }
    }

    // 使用Delaunay三角化来消除异常值
    std::vector<PointCloudT> delaunay_points;
    delaunay_points.reserve(m_angle_max_idx * m_radis_max_idx);
    std::vector<PointCloudT> oulier_points;
    // 抽取点云
    for(int i=0; i<m_angle_max_idx; ++i)
    {
        for(int j=0; j<m_radis_max_idx; ++j)
        {
            if(occ_map.at<int>(i,j) > 0)
            {
                PointCloudT temp_points;
                temp_points.pos_a = i;
                temp_points.pos_r = j;
                temp_points.x = m_cloth_terrain[i][j].GetPosX();
                temp_points.y = m_cloth_terrain[i][j].GetPosY();
                temp_points.z = height_map.at<float>(i,j);
                delaunay_points.push_back(temp_points);
            }
        }
    }
    // 迭代找到异常点
    for(int iter=0; iter<3; ++iter)
    {
        // 创建三角剖分
        IDelaBella* idb = IDelaBella::Create();
        int verts = idb->Triangulate(delaunay_points.size(), &delaunay_points[0].x, &delaunay_points[0].y, sizeof(PointCloudT));
        // 统计每个点的连接情况
        cv::Mat c_state = cv::Mat::zeros(delaunay_points.size(), 2, CV_32S);
        if (verts > 0)
        {
            int tris = verts / 3;
            float max_length = std::pow(m_kPolarGridmapParams.max_search_dis, 2) * 2;
            float max_slope = std::cos(m_kPolarGridmapParams.slope_thresh * ANGLE2RADIAN);
            const DelaBella_Triangle* dela = idb->GetFirstDelaunayTriangle();
            // 计算每个三角面片的坡度，大于定值则删除
            for (int i=0; i<tris; ++i)
            {
                float slope = CalculateTriangleSlope(delaunay_points, dela, max_length);
                if(slope < max_slope)
                {
                    c_state.at<int>(dela->v[0]->i,1)++;
                    c_state.at<int>(dela->v[1]->i,1)++;
                    c_state.at<int>(dela->v[2]->i,1)++;
                }
                else
                {
                    c_state.at<int>(dela->v[0]->i,0)++;
                    c_state.at<int>(dela->v[1]->i,0)++;
                    c_state.at<int>(dela->v[2]->i,0)++;
                }
                dela = dela->next;
            }
            // 保留地面点，剔除离群点
            std::vector<PointCloudT> remain_pc;
            remain_pc.reserve(delaunay_points.size());
            for(uint i=0; i<delaunay_points.size(); ++i)
            {
                if(c_state.at<int>(i,0) > c_state.at<int>(i,1))
                {
                    if(c_state.at<int>(i,0) > 1)
                    {
                        remain_pc.push_back(delaunay_points[i]);
                    }
                    else
                    {
                        oulier_points.push_back(delaunay_points[i]);
                    }
                }
                else
                {
                    oulier_points.push_back(delaunay_points[i]);
                }
            }
            idb->Destroy();
            if(remain_pc.size() == delaunay_points.size())
            {
                break;
            }
            else
            {
                delaunay_points = remain_pc;
            }
        }
    }

    for(int i=0; i<oulier_points.size(); ++i)
    {
        occ_map.at<int>(oulier_points[i].pos_a, oulier_points[i].pos_r) = 2;
    }

    // 通过径向搜索删除异常点
    for(int i=0; i<m_angle_max_idx; ++i)
    {
        // 盲区置占据，从非盲区开始搜索
        for(int curr_idx=0; curr_idx<=m_blind_area[i]; ++curr_idx)
        {
            occ_map.at<int>(i,curr_idx) = 1;
        }
        // 非盲区开始搜索
        int last_idx = m_blind_area[i];
        float last_z = 0;
        float last_r = m_radis_position[last_idx];
        int comp_idx = m_blind_area[i];
        float comp_z = 0;
        float comp_r = m_radis_position[comp_idx];
        for(int curr_idx=m_blind_area[i]+1; curr_idx<m_radis_max_idx; ++curr_idx)
        {
            // 如果点差距大于阈值则不再扩展退出本循环
            if(m_radis_position[curr_idx] - m_radis_position[comp_idx] > m_kPolarGridmapParams.max_search_dis)
            {
                for(int jj=curr_idx; jj<m_radis_max_idx; ++jj)
                {
                    occ_map.at<int>(i,jj) = 0;
                }
                break;
            }
            else
            {
                // 如果该点为地面占据点则判断梯度是否小于阈值，小于阈值扩展，否则删除
                float curr_r = m_radis_position[curr_idx];
                if(occ_map.at<int>(i,curr_idx) == 1)
                {
                    float curr_z = height_map.at<float>(i,curr_idx);
                    float slope = (curr_z - comp_z) / (curr_r - comp_r);
                    if(std::fabs(slope) < grad_thresh)
                    {
                        float dslope = (curr_z - last_z) / (curr_r - last_r);
                        for(int jj=last_idx+1; jj<curr_idx; ++jj)
                        {
                            occ_map.at<int>(i,jj) = 1;
                            height_map.at<float>(i,jj) = last_z + dslope * (m_radis_position[jj] - last_r);
                        }
                        last_r = curr_r;
                        last_z = curr_z;
                        last_idx = curr_idx;
                        comp_r = curr_r;
                        comp_z = curr_z;
                        comp_idx = curr_idx;
                    }
                    else
                    {
                        comp_r = curr_r;
                        comp_idx = curr_idx;
                        occ_map.at<int>(i,curr_idx) = 0;
                    }
                }
                // 如果该点为障碍点则直接跳过
                else if(occ_map.at<int>(i,curr_idx) == 2)
                {
//                    comp_r = curr_r;
//                    comp_idx = curr_idx;
                    occ_map.at<int>(i,curr_idx) = 0;
                }
            }
        }
    }

    // 使用上面的地面地形图给织物地形图赋值
    for(int i=0; i<m_angle_max_idx; ++i)
    {
        for(int j=0; j<m_radis_max_idx; ++j)
        {
            if(occ_map.at<int>(i,j) > 0)
            {
                m_cloth_terrain[i][j].SetOccupy(true, -height_map.at<float>(i,j));
            }
        }
    }
    // 抽取地面高度图
    cv::Mat occ_points = cv::Mat::zeros(0, 3, CV_32F);
    occ_points.reserve(m_radis_max_idx * m_angle_max_idx);
    for(int i=0; i<m_radis_max_idx; ++i)
    {
        for(int j=0; j<m_angle_max_idx; ++j)
        {
            if(m_cloth_terrain[j][i].GetOccupy())
            {
                cv::Mat temp_points = cv::Mat::zeros(1, 3, CV_32F);
                temp_points.at<float>(0,0) = m_cloth_terrain[j][i].GetPosX();
                temp_points.at<float>(0,1) = m_cloth_terrain[j][i].GetPosY();
                temp_points.at<float>(0,2) = m_cloth_terrain[j][i].GetGirdRefHeight();
                occ_points.push_back(temp_points);
            }
        }
    }



    // 抽取织物地形图中的占据点构造KDTREE，非占据点在此中寻找最近邻给自己赋值
    cv::flann::KDTreeIndexParams index_params(2);
    cv::flann::Index kdtree(occ_points.colRange(0,2).clone(), index_params);
    int query_num = 1;
    std::vector<float> vec_query(2);
    std::vector<int> vec_index(query_num);
    std::vector<float> vec_dist(query_num);
    cv::flann::SearchParams params(4);
    // 未占据的地形栅格从邻域找对应的高度
    for(int i=0; i<m_radis_max_idx; ++i)
    {
        for(int j=0; j<m_angle_max_idx; ++j)
        {
            if(!m_cloth_terrain[j][i].GetOccupy())
            {
                vec_query[0] = m_cloth_terrain[j][i].GetPosX();
                vec_query[1] = m_cloth_terrain[j][i].GetPosY();
                kdtree.knnSearch(vec_query, vec_index, vec_dist, query_num, params);
                if(vec_dist[0] < SEARCH_DIS)
                {
                    m_cloth_terrain[j][i].SetOccupy(true, occ_points.at<float>(vec_index[0],2));
                }
            }
        }
    }
}


float ObstacleMap::CalculateTriangleSlope(const std::vector<PointCloudT> &pc,
                                          const DelaBella_Triangle *dela,
                                          float max_length)
{
    // 计算边长
    float l1 = pow((dela->v[0]->x - dela->v[1]->x), 2) + pow((dela->v[0]->y - dela->v[1]->y), 2);
    float l2 = pow((dela->v[2]->x - dela->v[1]->x), 2) + pow((dela->v[2]->y - dela->v[1]->y), 2);
    float l3 = pow((dela->v[0]->x - dela->v[2]->x), 2) + pow((dela->v[0]->y - dela->v[2]->y), 2);
    float l_sum = l1 + l2 + l3;
    // 如果变长大于一定值则抛弃该三角形
    if(l_sum > max_length)
    {
        return 0;
    }
    // 计算三角形的坡度
    else
    {
        float x1 = dela->v[1]->x - dela->v[0]->x;
        float y1 = dela->v[1]->y - dela->v[0]->y;
        float z1 = pc[dela->v[1]->i].z - pc[dela->v[0]->i].z;
        float x2 = dela->v[2]->x - dela->v[0]->x;
        float y2 = dela->v[2]->y - dela->v[0]->y;
        float z2 = pc[dela->v[2]->i].z - pc[dela->v[0]->i].z;
        float vx = y1*z2 - z1*y2;
        float vy = -(x1*z2 - z1*x2);
        float vz = x1*y2 - y1*x2;
        float slope = std::fabs(vz) / std::sqrt(vx*vx + vy*vy + vz*vz);

        return slope;
    }
}


void ObstacleMap::TimeStep()
{
    int min_iter = std::min(10, m_kPolarGridmapParams.iterations);

    for(int it=0; it<m_kPolarGridmapParams.iterations; ++it)
    {
        // 直接力作用
        for(int i=0; i<m_radis_max_idx; ++i)
        {
            for(int j=0; j<m_angle_max_idx; ++j)
            {
                if(m_cloth_terrain[j][i].GetOccupy())
                {
                    m_cloth_terrain[j][i].DirectForceActions();
                }
            }
        }
        // 间接力作用运动
        for(int i=0; i<m_radis_max_idx; ++i)
        {
//#pragma omp parallel for
            for(int j=0; j<m_angle_max_idx; ++j)
            {
                m_cloth_terrain[j][i].IndirectForceActions(m_kPolarGridmapParams.rigidness);
            }
        }
        // 计算最大位移
        float max_move_dis = 0;
        for(int i=0; i<m_radis_max_idx; ++i)
        {
            for(int j=0; j<m_angle_max_idx; ++j)
            {
                float dis = m_cloth_terrain[j][i].GetMoveDis();
                if(dis > max_move_dis)
                {
                    max_move_dis = dis;
                }
            }
        }
        // 判断是否存在地形冲突
        TerrainCollision();
        // 判断是否满足最小移动阈值
        if(it > min_iter && max_move_dis < MIN_MOVE_DIS)
        {
            // 地形平滑
            TerrainSmooth();
            break;
        }
    }
}


void ObstacleMap::TerrainCollision()
{
    for(int i=0; i<m_radis_max_idx; ++i)
    {
        for(int j=0; j<m_angle_max_idx; ++j)
        {
            m_cloth_terrain[j][i].VerifyBelowReferenceHeight();
        }
    }
}


void ObstacleMap::TerrainSmooth()
{
    float grad_thresh = std::tan(m_kPolarGridmapParams.slope_thresh * ANGLE2RADIAN);
//    grad_thresh = grad_thresh * m_kPolarGridmapParams.radis_resolution;
    for(int i=0; i<m_angle_max_idx; ++i)
    {
        int last_idx = 0;
        float last_z = m_cloth_terrain[i][0].GetCurrZ();
        float last_r = m_radis_position[0];
        for(int j=1; j<m_radis_max_idx; ++j)
        {
            float curr_z = m_cloth_terrain[i][j].GetCurrZ();
            float curr_r = m_radis_position[j];
            float slope = (curr_z - last_z) / (curr_r - last_r);
            if(std::fabs(slope) < grad_thresh)
            {
                for(int k=last_idx+1; k<j; ++k)
                {
                    float interp_z = slope * (m_radis_position[k] - last_r) + last_z;
                    m_cloth_terrain[i][k].SetCurrZ(-interp_z);
                }
                last_idx = j;
                last_z = curr_z;
                last_r = curr_r;
            }
        }
    }
}


void ObstacleMap::UpdatePointTypeByGround()
{
    // 用点和地面差的高度来判断点的类型
    for(int i=0; i<m_valid_num; ++i)
    {
        if(m_pointclouds[i].type < 3)
        {
            int a_idx = m_pointclouds[i].pos_a;
            int r_idx = m_pointclouds[i].pos_r;
            if(m_cloth_terrain[a_idx][r_idx].GetOccupy())
            {
                float dis_z = m_pointclouds[i].z - m_cloth_terrain[a_idx][r_idx].GetCurrZ();
                float inc_z = m_radis_position[r_idx] * m_kPolarGridmapParams.height_increase;
                float comp_z = m_kPolarGridmapParams.height_thresh + inc_z;
                if(dis_z < -comp_z)
                {
                    m_pointclouds[i].type = 2;
                }
                else if(dis_z < comp_z)
                {
                    m_pointclouds[i].type = 0;
                }
                else if(dis_z < m_kCarModelParams.suspended_height)
                {
                    m_pointclouds[i].type = 1;
                }
                else
                {
                    m_pointclouds[i].type = 3;
                }
            }
            else
            {
                m_pointclouds[i].type = 3;
            }
        }
        m_pointclouds[i].z = m_pointclouds[i].z - m_kCarModelParams.ground_height;
    }
}


void ObstacleMap::ObstalePointFilter()
{
    // 迭代循环
    TimeStep();
    // 使用估计的地形来更新点的属性
    UpdatePointTypeByGround();
}
