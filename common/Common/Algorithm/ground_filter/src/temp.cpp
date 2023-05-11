////#######################################################################################
////#  This source code is about a ground filtering algorithm for airborn LiDAR data      #
////#  based on physical process simulations, specifically cloth simulation.              #
////#                                     Copyright 2021                                  #
////#                  Changsha Xingshen Intelligent Technology Co., Ltd                  #
////#                               (http://www.xingshentech.com/)                        #
////#                                                                                     #
////#                                  Authored By Meng Deyuan                            #
////#                                                                                     #
////#######################################################################################

//#include "obstaclemap.h"


//ObstacleMap::ObstacleMap(CarModelParams car_params, PolarGridmapParams polar_params):
//    m_kCarModelParams(car_params),
//    m_kPolarGridmapParams(polar_params)
//{
//    InitPolarGridmapParams();
//    for(int i=0; i<ATAN_LIST_NUM; ++i)
//    {
//        int angle = int(std::atan(double(i) / 100.0) * RADIAN2ANGLE);
//        m_atan_angle.push_back(angle);
//    }
//}


//void ObstacleMap::InitPolarGridmapParams()
//{
//    // 1. 计算径向和角向的最大值，角度范围为：[angle_min, angle_max)
//    m_radis_max_idx = int(std::round(m_kPolarGridmapParams.radis_range / m_kPolarGridmapParams.radis_resolution));
//    int idx_min = int(std::floor(float(m_kPolarGridmapParams.angle_range_min) / m_kPolarGridmapParams.angle_resolution));
//    int idx_max = int(std::ceil(float(m_kPolarGridmapParams.angle_range_max) / m_kPolarGridmapParams.angle_resolution));
//    m_angle_max_idx = idx_max - idx_min;
//    // 2. 计算径向栅格的中心位置
//    m_radis_position.resize(m_radis_max_idx);
//    for(int i=0; i<m_radis_max_idx; ++i)
//    {
//        m_radis_position[i] = (float(i) + 0.5) * m_kPolarGridmapParams.radis_resolution;
//    }
//    // 3. 计算每个水平角片区的盲区区域，盲区区域是指由于车身占据位置导致的径向不可见盲区
//    //    m_blind_area.resize(m_angle_max_idx);
//    int round_idx = int(std::round(360.0f / m_kPolarGridmapParams.angle_resolution));
//    std::vector<int> blind_area(round_idx);
//    int angle1;
//    int angle2;
//    float k;
//    float r;
//    // 3.1 后盲区边界，即左后角点和右后角点之间的盲区的极坐标表示
//    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 180.0));
//    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 540.0));
//    for(int i=angle1; i<=angle2; ++i)
//    {
//        k = std::tan((-i + 270) * ANGLE2RADIAN);
//        r = std::fabs(m_kCarModelParams.boundary_rear) * std::sqrt(1 / std::pow(k, 2) + 1);
//        int search_idx = i % 360;
//        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
//    }
//    // 3.2 左盲区边界，即左后角点和左前角点之间的盲区的极坐标表示
//    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 180.0));
//    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
//    for(int i=angle1; i<=angle2; ++i)
//    {
//        k = std::tan((-i + 270) * ANGLE2RADIAN);
//        r = std::fabs(m_kCarModelParams.boundary_left) * std::sqrt(std::pow(k, 2) + 1);
//        int search_idx = i % 360;
//        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
//    }
//    // 3.3 前盲区边界，即左前角点和右前角点之间的盲区的极坐标表示
//    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_left, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
//    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
//    for(int i=angle1; i<=angle2; ++i)
//    {
//        k = std::tan((-i + 270) * ANGLE2RADIAN);
//        r = std::fabs(m_kCarModelParams.boundary_front) * std::sqrt(1 / std::pow(k, 2) + 1);
//        int search_idx = i % 360;
//        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
//    }
//    // 3.4 右盲区边界，即右后角点和右前角点之间的盲区的极坐标表示
//    angle1 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_front) * RADIAN2ANGLE + 180.0));
//    angle2 = int(std::round(std::atan2(m_kCarModelParams.boundary_right, m_kCarModelParams.boundary_rear) * RADIAN2ANGLE + 180.0));
//    for(int i=angle1; i<=angle2; ++i)
//    {
//        k = std::tan((-i + 270) * ANGLE2RADIAN);
//        r = std::fabs(m_kCarModelParams.boundary_right) * std::sqrt(std::pow(k, 2) + 1);
//        int search_idx = i % 360;
//        blind_area[search_idx/m_kPolarGridmapParams.angle_resolution] = int(std::round(r / m_kPolarGridmapParams.radis_resolution));
//    }
//    // 3.5 约束最小盲区为1
//    for(uint i=0; i<blind_area.size(); ++i)
//    {
//        if(blind_area[i] < 1)
//        {
//            blind_area[i] = 1;
//        }
//        else if(blind_area[i] >= m_radis_max_idx - 1)
//        {
//            blind_area[i] = m_blind_area[i] - 1;
//        }
//    }
//    for(int i=idx_min; i<idx_max; ++i)
//    {
//        int ii = i % blind_area.size();
//        m_blind_area.push_back(blind_area[ii]);
//    }
//    // 4. 初始化模拟地形
//    // 4.1 创建地形
//    m_cloth_terrain.resize(m_angle_max_idx);
//    for(int i=0; i<m_angle_max_idx; ++i)
//    {
//        m_cloth_terrain[i].resize(m_radis_max_idx);
//    }
//    // 4.2 创建连接
//    bool round_flag = false;
//    if(m_kPolarGridmapParams.angle_range_min == 0 && m_kPolarGridmapParams.angle_range_max == 360)
//    {
//        round_flag = true;
//    }
//    for(int i=0; i<m_radis_max_idx; ++i)
//    {
//        for(int j=0; j<m_angle_max_idx; ++j)
//        {
//            for(int ii=i-2; ii<i+3; ++ii)
//            {
//                for(int jj=j-2; jj<j+3; ++jj)
//                {
//                    if(ii == i && jj == j)
//                    {
//                        continue;
//                    }
//                    if(ii >= 0 && ii < m_radis_max_idx)
//                    {
//                        if(round_flag)
//                        {
//                            int temp_jj = (jj + m_angle_max_idx) % m_angle_max_idx;
//                            m_cloth_terrain[j][i].AddNeighbor(&m_cloth_terrain[temp_jj][ii]);
//                        }
//                        else
//                        {
//                            if(jj >= 0 && jj < m_angle_max_idx)
//                            {
//                                m_cloth_terrain[j][i].AddNeighbor(&m_cloth_terrain[jj][ii]);
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//}


//int ObstacleMap::CountAtanAngle(float x, float y)
//{
//    int angle;
//    int idx;
//    float tan_v;
//    if(y == 0)
//    {
//        if(x >= 0)
//        {
//            angle = 270;
//        }
//        else
//        {
//            angle = 90;
//        }
//    }
//    else if(y > 0)
//    {
//        tan_v = 100 * x / y;
//        if(tan_v >= 0)
//        {
//            idx = int(tan_v);
//            angle = GetAtanAngleFromList(idx) + 180;

//        }
//        else
//        {
//            idx = int(-tan_v);
//            angle = -GetAtanAngleFromList(idx) + 180;
//        }
//    }
//    else
//    {
//        tan_v = 100 * x / y;
//        if(tan_v >= 0)
//        {
//            idx = int(tan_v);
//            angle = GetAtanAngleFromList(idx);

//        }
//        else
//        {
//            idx = int(-tan_v);
//            angle = -GetAtanAngleFromList(idx) + 360;
//            angle = angle % 360;
//        }
//    }

//    return angle;
//}


//int ObstacleMap::GetAtanAngleFromList(int x)
//{
//    int angle;
//    if(x >= ATAN_LIST_NUM)
//    {
//        angle = 90;
//    }
//    else
//    {
//        angle = m_atan_angle[x];
//    }

//    return angle;
//}


//void ObstacleMap::PointCloudPreprocess(cv::Mat &lidar_data)
//{
//    m_valid_num = 0;
//    m_pointclouds.clear();
//    m_pointclouds.resize(lidar_data.rows);
//    // 迪卡尔坐标系转极坐标
//    float x;
//    float y;
//    float z;
//    int angle;
//    float distance;
//    float min_z = MAX_INF;
//    for(int i=0; i<lidar_data.rows; ++i)
//    {
//        x = lidar_data.at<float>(i,0);
//        y = lidar_data.at<float>(i,1);
//        z = lidar_data.at<float>(i,2);
//        if(x != 0 || y != 0)
//        {
//            angle = CountAtanAngle(x, y) - m_kPolarGridmapParams.angle_range_min;
//            if(angle < 0)
//            {
//                angle = angle + 360;
//            }
//            m_pointclouds[m_valid_num].x = x;
//            m_pointclouds[m_valid_num].y = y;
//            m_pointclouds[m_valid_num].z = z;
//            m_pointclouds[m_valid_num].pos_a = angle / m_kPolarGridmapParams.angle_resolution;
//            distance = std::sqrt(std::pow(x,2) + std::pow(y,2));
//            m_pointclouds[m_valid_num].pos_r =int(distance / m_kPolarGridmapParams.radis_resolution);
//            if(m_pointclouds[m_valid_num].pos_a >= m_angle_max_idx ||
//                    m_pointclouds[m_valid_num].pos_r >= m_radis_max_idx ||
//                    m_pointclouds[m_valid_num].z < MINZ)
//            {
//                m_pointclouds[m_valid_num].type = 3;
//            }
//            else
//            {
//                // 记录有效点的最低点高度
//                if(z < min_z)
//                {
//                    min_z = z;
//                }
//            }
//            m_valid_num++;
//        }
//    }
//    RasterTerrain(-min_z + CLOTH_Z_COMP);
//}


//void ObstacleMap::RasterTerrain(float max_h)
//{
//    // 织物地形图初始化
//    float time_step2 = m_kPolarGridmapParams.time_step * m_kPolarGridmapParams.time_step;
//    for(int i=0; i<m_radis_max_idx; ++i)
//    {
//        for(int j=0; j<m_angle_max_idx; ++j)
//        {
//            m_cloth_terrain[j][i].Init(max_h, time_step2);
//        }
//    }
//    // 有效点云向织物地形图投影
//    int idx_r;
//    int idx_a;
//    for(int i=0; i<m_valid_num; ++i)
//    {
//        if(m_pointclouds[i].type < 3)
//        {
//            idx_r = m_pointclouds[i].pos_r;
//            idx_a = m_pointclouds[i].pos_a;
//            m_cloth_terrain[idx_a][idx_r].SetOccupy(true, -m_pointclouds[i].z);
//        }
//    }
//    // 限制区域投影
//    for(int i=0; i<m_angle_max_idx; ++i)
//    {
//        for(int j=0; j<m_blind_area[i]; ++j)
//        {
//            m_cloth_terrain[i][j].SetOccupy(true, 0);
//        }
//    }
//    // 未占据的地形栅格从邻域找对应的高度
//    int num0 = 0;
//    int num1 = 0;
//    for(int i=0; i<m_radis_max_idx; ++i)
//    {
//        for(int j=0; j<m_angle_max_idx; ++j)
//        {
//            if(!m_cloth_terrain[j][i].GetOccupy())
//            {
//                float height_val = FindHeightValByNeighbor(&m_cloth_terrain[j][i]);
//                m_cloth_terrain[j][i].SetOccupy(false, height_val);
//                num1++;
//            }
//            else
//            {
//                num0++;
//            }
//        }
//    }
//    std::cout << "Project:" <<  num0 << " " << num1 << std::endl;
//}


//float ObstacleMap::FindHeightValByNeighbor(Particle *p)
//{
//    std::queue<Particle*> nqueue;
//    std::vector<Particle*> pbacklist;
//    size_t neiborsize = p->GetNeighborNum();
//    for (size_t i=0; i<neiborsize; ++i)
//    {
//        p->SetVisited(true);
//        nqueue.push(p->GeiNeighborByIndex(i));
//    }
//    // 递归迭代
//    while (!nqueue.empty())
//    {
//        Particle *pneighbor = nqueue.front();
//        nqueue.pop();
//        pbacklist.push_back(pneighbor);
//        if (pneighbor->GetOccupy())
//        {
//            // 将已访问列表中的粒子都置为未访问
//            for (int i=0; i<pbacklist.size(); ++i)
//            {
//                pbacklist[i]->SetVisited(false);
//            }
//            // 将待访问列表中的粒子都置为未访问
//            while (!nqueue.empty())
//            {
//                Particle *pp = nqueue.front();
//                pp->SetVisited(false);
//                nqueue.pop();
//            }
//            return pneighbor->GetGirdRefHeight();
//        }
//        else
//        {
//            int nsize = pneighbor->GetNeighborNum();
//            for (int i=0; i<nsize; ++i)
//            {
//                Particle *ptmp = pneighbor->GeiNeighborByIndex(i);
//                if (!ptmp->GetVisited())
//                {
//                    ptmp->SetVisited(true);
//                    nqueue.push(ptmp);
//                }
//            }
//        }
//    }

//    return -MINZ + CLOTH_Z_COMP;
//}


//void ObstacleMap::TimeStep()
//{
//    int min_iter = std::min(10, m_kPolarGridmapParams.iterations);
//    for(int i=0; i<m_kPolarGridmapParams.iterations; ++i)
//    {
//        // 直接力作用
//        //#pragma omp parallel for
//        for(int i=0; i<m_radis_max_idx; ++i)
//        {
//            for(int j=0; j<m_angle_max_idx; ++j)
//            {
//                m_cloth_terrain[j][i].DirectForceActions();
//            }
//        }
//        // 间接力作用运动
//        //#pragma omp parallel for
//        for(int i=0; i<m_radis_max_idx; ++i)
//        {
//            for(int j=0; j<m_angle_max_idx; ++j)
//            {
//                m_cloth_terrain[j][i].IndirectForceActions(m_kPolarGridmapParams.rigidness);
//            }
//        }

//        float max_move_dis = 0;
//        float dis;
//        // 计算delta_z
//        for(int i=0; i<m_radis_max_idx; ++i)
//        {
//            for(int j=0; j<m_angle_max_idx; ++j)
//            {
//                dis = m_cloth_terrain[j][i].GetMoveDis();
//                if(dis > max_move_dis)
//                {
//                    max_move_dis = dis;
//                }
//            }
//        }
//        std::cout << i << " " << max_move_dis << std::endl;
//        // 判断是否存在地形冲突
//        TerrainCollision();
//        // 判断是否满足最小移动阈值
//        if(i > min_iter && max_move_dis < MIN_MOVE_DIS)
//        {
//            break;
//        }
//    }
//}


//void ObstacleMap::TerrainCollision()
//{
//    //#pragma omp parallel for
//    for(int i=0; i<m_radis_max_idx; ++i)
//    {
//        for(int j=0; j<m_angle_max_idx; ++j)
//        {
//            m_cloth_terrain[j][i].VerifyBelowReferenceHeight();
//        }
//    }
//}


//void ObstacleMap::ObstalePointFilter()
//{
//    // 迭代循环
//    TimeStep();
//    // 获取地面高度
//    cv::Mat ground_height = cv::Mat::zeros(m_angle_max_idx, m_radis_max_idx, CV_32F);
//    cv::Mat output_height = cv::Mat::zeros(m_angle_max_idx, m_radis_max_idx, CV_32F);
//    for(int i=0; i<m_angle_max_idx; ++i)
//    {
//        for(int j=0; j<m_radis_max_idx; ++j)
//        {
//            ground_height.at<float>(i,j) = m_cloth_terrain[i][j].GetCurrZ();
//        }
//    }
//    // 地面滤波
//    for(int i=0; i<m_radis_max_idx; ++i)
//    {
//        for(int j=135; j<136; ++j)
//        {
//            std::cout << j << " " << i << " " << m_cloth_terrain[j][i].GetCurrZ() << std::endl;
//        }
//    }
//    float grad_thresh = std::tan(m_kPolarGridmapParams.slope_thresh * ANGLE2RADIAN);
//    PolyfitCenterGround(ground_height, output_height, grad_thresh, 135);
//    // 用点和地面差的高度来判断点的类型
//    int a_idx;
//    int r_idx;
//    float dis_z;
//    float comp_z;
//    float inc_z;
//    for(int i=0; i<m_valid_num; ++i)
//    {
//        if(m_pointclouds[i].type < 3)
//        {
//            a_idx = m_pointclouds[i].pos_a;
//            r_idx = m_pointclouds[i].pos_r;
//            dis_z = m_pointclouds[i].z - m_cloth_terrain[a_idx][r_idx].GetCurrZ();
//            inc_z = (m_radis_position[r_idx] - 5.0) * m_kPolarGridmapParams.height_increase;
//            if(inc_z < 0)
//            {
//                inc_z = 0;
//            }
//            comp_z = m_kPolarGridmapParams.height_thresh + inc_z;
//            if(dis_z < -comp_z)
//            {
//                m_pointclouds[i].type = 2;
//            }
//            else if(dis_z < comp_z)
//            {
//                m_pointclouds[i].type = 0;
//            }
//            else if(dis_z < m_kCarModelParams.suspended_height)
//            {
//                m_pointclouds[i].type = 1;
//            }
//            else
//            {
//                m_pointclouds[i].type = 3;
//            }
//        }
//    }


//}


//void ObstacleMap::UpdatePointTypeByFitGround()
//{
////    // 2. 将盲区区域占据置1, 高度置0
////    cv::Mat fit_occ = cv::Mat::zeros(m_angle_max_idx, m_radis_max_idx, CV_32S);
////    cv::Mat fit_height = cv::Mat::zeros(m_angle_max_idx, m_radis_max_idx, CV_32F);
////    for(int i=0; i<m_angle_max_idx; ++i)
////    {
////        if(i % INTERPOLATE_INTERVAL == 0)
////        {
////            PolyfitCenterGround(gridmap_occ, gridmap_min, fit_occ, fit_height, i);
////        }
////    }
//}



////void ObstacleMap::NoiseFliter()
////{
////    // 1. 创建极坐标统计地图，滤除噪点
////    // 高度为[-1,4]，高度的分辨率为20cm
////    int a_idx;
////    int r_idx;
////    int z_idx;
////    int idx;
////    float max_height = std::max(m_kCarModelParams.suspended_height+1.0f, COMP_MAX_HEIGHT);
////    int z_max_idx = int(std::ceil((max_height + m_kCarModelParams.suspended_height) / 0.2f));
////    int size[3] = {z_max_idx, m_angle_max_idx, m_radis_max_idx};
////    cv::Mat noise_count = cv::Mat::zeros(3, size, CV_32S);
////    cv::Mat noise_occ = cv::Mat::zeros(3, size, CV_32S);
////    int step0 = noise_count.step[0];
////    int step1 = noise_count.step[1];
////    int step2 = noise_count.step[2];
////    int *noise_sum;
////    int *noise_flag;
////    // 2. 障碍点投影
////    float x;
////    float y;
////    for(int i=0; i<m_valid_num; ++i)
////    {
////        // 如果障碍点在车体内则删除
////        if(m_pointclouds[i].type == 1)
////        {
////            x = m_pointclouds[i].x;
////            y = m_pointclouds[i].y;
////            if(x > m_kCarModelParams.boundary_left && x < m_kCarModelParams.boundary_right &&
////               y > m_kCarModelParams.boundary_rear && y < m_kCarModelParams.boundary_front)
////            {
////                m_pointclouds[i].type = 3;
////            }
////            else
////            {
////                a_idx = m_pointclouds[i].pos_a;
////                r_idx = m_pointclouds[i].pos_r;
////                z_idx = int((m_pointclouds[i].z + m_kCarModelParams.suspended_height) / 0.2f);
////                idx = step0 * z_idx + step1 * a_idx + step2 * r_idx;
////                noise_sum = (int *)(noise_count.data + idx);
////                *noise_sum += 1;
////            }
////        }
////    }
////    // 计算噪声点个数
////    for(int i=0; i<m_valid_num; ++i)
////    {
////        if(m_pointclouds[i].type == 1)
////        {
////            a_idx = m_pointclouds[i].pos_a;
////            r_idx = m_pointclouds[i].pos_r;
////            z_idx = int((m_pointclouds[i].z + m_kCarModelParams.suspended_height) / 0.2f);
////            idx = step0 * z_idx + step1 * a_idx + step2 * r_idx;
////            noise_flag = (int *)(noise_occ.data + idx);
////            if(*noise_flag == 0)
////            {
////                noise_sum = (int *)(noise_count.data + idx);
////                // 如果障碍点小于噪声阈值，则判断立方范围和垂直范围障碍点个数是否符合条件
////                if(*noise_sum < m_kPolarGridmapParams.noise_num)
////                {
////                    int temp_aa;
////                    int square_sum = 0;
////                    for(int zz=z_idx-1; zz<z_idx+2; ++zz)
////                    {
////                        if(zz < 0 || zz >= z_max_idx)
////                        {
////                            continue;
////                        }
////                        for(int aa=a_idx-2; aa<a_idx+3; ++aa)
////                        {
////                            if(aa >= m_angle_max_idx)
////                            {
////                                temp_aa = aa - m_angle_max_idx;
////                            }
////                            else if(aa < 0)
////                            {
////                                temp_aa = aa + m_angle_max_idx;
////                            }
////                            else
////                            {
////                                temp_aa = aa;
////                            }
////                            for(int rr=r_idx-2; rr<r_idx+3; ++rr)
////                            {
////                                if(rr < 0 || rr >= m_radis_max_idx)
////                                {
////                                    continue;
////                                }
////                                idx = step0 * zz + step1 * temp_aa + step2 * rr;
////                                noise_sum = (int*)(noise_count.data + idx);
////                                square_sum += *noise_sum;
////                            }
////                        }
////                    }
////                    if(square_sum < 2 * m_kPolarGridmapParams.noise_num)
////                    {
////                        int pillar_sum = 0;
////                        for(int zz=0; zz<z_max_idx; ++zz)
////                        {
////                            for(int aa=a_idx-1; aa<a_idx+2; ++aa)
////                            {
////                                if(aa >= m_angle_max_idx)
////                                {
////                                    temp_aa = aa - m_angle_max_idx;
////                                }
////                                else if(aa < 0)
////                                {
////                                    temp_aa = aa + m_angle_max_idx;
////                                }
////                                else
////                                {
////                                    temp_aa = aa;
////                                }
////                                for(int rr=r_idx-1; rr<r_idx+2; ++rr)
////                                {
////                                    if(rr < 0 || rr >= m_radis_max_idx)
////                                    {
////                                        continue;
////                                    }
////                                    idx = step0 * zz + step1 * temp_aa + step2 * rr;
////                                    noise_sum = (int*)(noise_count.data + idx);
////                                    pillar_sum += *noise_sum;
////                                }
////                            }
////                        }
////                        if(pillar_sum < 2 * m_kPolarGridmapParams.noise_num)
////                        {
////                            *noise_flag = 1;
////                            m_pointclouds[i].type = 3;
////                        }
////                    }
////                }
////            }
////            else
////            {
////                m_pointclouds[i].type = 3;
////            }
////        }
////    }
////}


////void ObstacleMap::GetPositiveObstaclePoints()
////{
////    m_obstacle_points.release();
////    cv::Mat obstacle_point = cv::Mat::zeros(1,3,CV_32F);
////    for(int i=0; i<m_valid_num; ++i)
////    {
////        if(m_pointclouds[i].type == 1)
////        {
////            obstacle_point.at<float>(0,0) = m_pointclouds[i].x;
////            obstacle_point.at<float>(0,1) = m_pointclouds[i].y;
////            obstacle_point.at<float>(0,2) = m_pointclouds[i].z;
////            m_obstacle_points.push_back(obstacle_point);
////        }
////    }
////}


//float ObstacleMap::CalculateTempPostionValue(std::vector<float> &params, float px)
//{
//    int params_len = params.size();
//    float pz = 0;
//    for(int i=1; i<=params_len; ++i)
//    {
//        pz = pz + params[i-1] * std::pow(px, i);
//    }
//    return pz;
//}


//float ObstacleMap::CalculateSlope(float upper_r, float upper_z, float curr_r, float curr_z)
//{
//    return std::fabs((curr_z - upper_z) / (curr_r - upper_r));
//}


//void ObstacleMap::Polyfit(cv::Mat &ground_points, std::vector<float> &params, int poly_idx)
//{
//    cv::Mat x = ground_points.colRange(1, 1+poly_idx);
//    cv::Mat y = ground_points.col(0);
//    cv::Mat xt;
//    cv::transpose(x, xt);
//    cv::Mat xxi;
//    cv::invert(xt * x, xxi);
//    cv::Mat a;
//    a = xxi * xt * y;
//    if(std::isinf(a.at<float>(0,0)) == false && std::isnan(a.at<float>(0,0)) == false)
//    {
//        params.clear();
//        for(int i=0; i<poly_idx; ++i)
//        {
//            params.push_back(a.at<float>(i,0));
//        }
//    }
//}


//void ObstacleMap::PolyfitCenterGround(cv::Mat &input_heightmap,
//                                      cv::Mat &output_heightmap,
//                                      float grad_thresh,
//                                      int idx)
//{

//    cv::Mat temp_ground = cv::Mat::zeros(1, 5, CV_32F);
//    int last_find = 0;
//    std::vector<float> params(1,0);
//    // 循环通过判断是否满足梯度阈值来进行滤波
//    for(int j=1; j<m_radis_max_idx; ++j)
//    {
//        float upper_r = m_radis_position[j-1];
//        float upper_z = CalculateTempPostionValue(params, upper_r);
//        float curr_z = input_heightmap.at<float>(idx,j);
//        float curr_r = m_radis_position[j];
//        // 计算当前位置点和上一个有效点的坡度角
//        float slope = CalculateSlope(upper_r, upper_z, curr_r, curr_z);
//        if(slope < grad_thresh)
//        {
//            // 添加地面点
//            cv::Mat temp_v = cv::Mat::zeros(1, 5, CV_32F);
//            temp_v.at<float>(0,0) = curr_z;
//            temp_v.at<float>(0,1) = curr_r;
//            temp_v.at<float>(0,2) = curr_r * temp_v.at<float>(0,1);
//            temp_v.at<float>(0,3) = curr_r * temp_v.at<float>(0,2);
//            temp_v.at<float>(0,4) = curr_r * temp_v.at<float>(0,3);
//            temp_ground.push_back(temp_v);
//            // 计算拟合的多项式次数
//            last_find = j;
//            int poly_idx;
//            if(m_radis_position[j] < m_poly1_border)
//            {
//                poly_idx = 1;
//            }
//            else if(m_radis_position[j] < m_poly2_border)
//            {
//                poly_idx = 2;
//            }
//            else if(m_radis_position[j] < m_poly3_border)
//            {
//                poly_idx = 3;
//            }
//            else
//            {
//                poly_idx = 4;
//            }
//            // 由地面点计算多项式方程
//            Polyfit(temp_ground, params, poly_idx);
//        }
//    }
//    std::cout << last_find << std::endl;
//    std::cout << params[0] << " " << params[1] << " " << params[2] << " " << params[3] << std::endl;
////    float curr_k = 0;
////    float curr_z = 0;
////    int expand_idx = 0;
////    // 进行有效点地面插值
////    for(int j=0; j<m_blind_area[idx]; ++j)
////    {
////        output_occmap.at<int>(idx,j) = 1;
////        output_heightmap.at<float>(idx,j) = 0;
////    }
////    for(int j=m_blind_area[idx]; j<=last_find; ++j)
////    {
////        CalculateTempPostionValue(params, m_radis_position[j], &curr_k, &curr_z);
////        if(std::fabs(curr_k) < m_grad_thresh)
////        {
////            output_occmap.at<int>(idx,j) = 1;
////            output_heightmap.at<float>(idx,j) = curr_z;
////            expand_idx = j;
////        }
////        else
////        {
////            break;
////        }
////    }
//}








////cv::Mat ObstacleMap::ObstalePointToObstacleMap(cv::Mat &obstacle_points)
////{
////    cv::Mat gridmap = cv::Mat::zeros(m_gridmap_h, m_gridmap_w, CV_32S);
////    // 将地面区域障碍点向占据栅格地图投影
////    int x_idx;
////    int y_idx;
////    float move_x;
////    float move_y;
////    for(int i=0; i<obstacle_points.rows; ++i)
////    {
////        move_x = obstacle_points.at<float>(i,0) - m_kCartsGridmapParams.min_x;
////        move_y = obstacle_points.at<float>(i,1) - m_kCartsGridmapParams.min_y;
////        x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////        y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////        if(x_idx >= 0 && x_idx < m_gridmap_h && y_idx >= 0 && y_idx < m_gridmap_w)
////        {
////            gridmap.at<int>(x_idx,y_idx) = 1;
////        }
////    }

////    return gridmap;
////}


////void ObstacleMap::ShowOutput()
////{
////    // 设置显示图像
////    cv::Mat showimg = cv::Mat::zeros(m_gridmap_h, m_gridmap_w, CV_8UC3);
////    for(int i=0; i<m_gridmap_h; ++i)
////    {
////        for(int j=0; j<m_gridmap_w; ++j)
////        {
////            if(m_prob_gridmap.at<uchar>(i,j) > 0)
////            {
////                showimg.at<cv::Vec3b>(i,j) = {0,0,255};
////            }
////        }
////    }
////    float res_xy = m_kCartsGridmapParams.res_xy;
////    // 显示距离参考线
////    int r05 = int(5.0f / res_xy);
////    int r10 = int(10.0f / res_xy);
////    int r20 = int(20.0f / res_xy);
////    int r30 = int(30.0f / res_xy);
////    int cx = int(-m_kCartsGridmapParams.min_x / res_xy);
////    int cy = int(-m_kCartsGridmapParams.min_y / res_xy);
////    cv::circle(showimg, cv::Point(cy, cx), r05, cv::Scalar(128, 128, 128));
////    cv::circle(showimg, cv::Point(cy, cx), r10, cv::Scalar(128, 128, 128));
////    cv::circle(showimg, cv::Point(cy, cx), r20, cv::Scalar(128, 128, 128));
////    cv::circle(showimg, cv::Point(cy, cx), r30, cv::Scalar(128, 128, 128));

////    //显示车体区域
////    cv::Scalar color = cv::Scalar(255, 255, 255);
////    cv::Point p0;
////    cv::Point p1;
////    int x_idx;
////    int y_idx;
////    float move_x;
////    float move_y;
////    // 后边线
////    move_x = m_kCarModelParams.boundary_left - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_rear - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p0.y = x_idx;
////    p0.x = y_idx;
////    move_x = m_kCarModelParams.boundary_right - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_rear - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p1.y = x_idx;
////    p1.x = y_idx;
////    cv::line(showimg, p0, p1, color, 2);
////    // 右边线
////    move_x = m_kCarModelParams.boundary_right - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_rear - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p0.y = x_idx;
////    p0.x = y_idx;
////    move_x = m_kCarModelParams.boundary_right - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_front - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p1.y = x_idx;
////    p1.x = y_idx;
////    cv::line(showimg, p0, p1, color, 2);
////    // 上边线
////    move_x = m_kCarModelParams.boundary_right - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_front - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p0.y = x_idx;
////    p0.x = y_idx;
////    move_x = m_kCarModelParams.boundary_left - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_front - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p1.y = x_idx;
////    p1.x = y_idx;
////    cv::line(showimg, p0, p1, color, 2);
////    // 左边线
////    move_x = m_kCarModelParams.boundary_left - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_front - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p0.y = x_idx;
////    p0.x = y_idx;
////    move_x = m_kCarModelParams.boundary_left - m_kCartsGridmapParams.min_x;
////    move_y = m_kCarModelParams.boundary_rear - m_kCartsGridmapParams.min_y;
////    x_idx = int(std::round(move_x / m_kCartsGridmapParams.res_xy));
////    y_idx = int(std::round(move_y / m_kCartsGridmapParams.res_xy));
////    p1.y = x_idx;
////    p1.x = y_idx;
////    cv::line(showimg, p0, p1, color, 2);
////    // 显示实例目标
////    color = cv::Scalar(0, 255, 0);
//////    for(uint i=0; i<m_object_border.size(); ++i)
//////    {
//////        for(uint j=0; j<m_object_border[i].size(); ++j)
//////        {
//////            int jj = (j+1) % m_object_border[i].size();
//////            p0.y = m_object_border[i][j].x;
//////            p0.x = m_object_border[i][j].y;
//////            p1.y = m_object_border[i][jj].x;
//////            p1.x = m_object_border[i][jj].y;
//////            cv::line(showimg, p0, p1, color, 2);
//////        }
//////    }
////    for(int i=0; i<m_static_objects.rows; ++i)
////    {
////        p0.y = int(std::round((m_static_objects.at<float>(i,0) - m_kCartsGridmapParams.min_x) / res_xy));
////        p0.x = int(std::round((m_static_objects.at<float>(i,1) - m_kCartsGridmapParams.min_y) / res_xy));
////        cv::circle(showimg, p0, 2, color);
////    }
////    cv::transpose(showimg, showimg);
////    cv::flip(showimg, showimg, 0);
////    cv::imshow("mw", showimg);
//////    cv::imwrite("./img/"+std::to_string(int(m_localpose_list[0].ctime))+".bmp", tf_img);
////    cv::waitKey(1);
////}


////void ObstacleMap::ObstaleStaticPoints()
////{
////    m_static_objects.release();
////    cv::Mat static_object = cv::Mat::zeros(1,2,CV_32F);
////    cv::Mat track_ins_map = cv::Mat::zeros(m_gridmap_h, m_gridmap_w, CV_8U);
////    int x_idx;
////    int y_idx;
////    float res_xy = m_kCartsGridmapParams.res_xy;
////    for(std::vector<cv::Mat>::iterator it=m_search_lines.begin(); it!=m_search_lines.end(); ++it)
////    {
////        for(int j=0; j<(*it).rows; ++j)
////        {
////            x_idx = (*it).at<int>(j,0);
////            y_idx = (*it).at<int>(j,1);
////            if(m_prob_gridmap.at<uchar>(x_idx,y_idx) == 1)
////            {
////                if(track_ins_map.at<uchar>(x_idx,y_idx) == 1)
////                {
////                    break;
////                }
////                else
////                {
////                    track_ins_map.at<uchar>(x_idx,y_idx) = 1;
////                    static_object.at<float>(0,0) = float(x_idx) * res_xy + m_kCartsGridmapParams.min_x;
////                    static_object.at<float>(0,1) = float(y_idx) * res_xy + m_kCartsGridmapParams.min_y;
////                    m_static_objects.push_back(static_object);
////                }
////                break;
////            }
////        }
////    }
////}


////cv::Mat ObstacleMap::DeletePointsInObjects(cv::Mat &track_ins_map, cv::Mat &obstacle_points)
////{
////    // 遍历所有障碍点，删除在检测目标内的障碍点
////    float move_x;
////    float move_y;
////    int x_idx;
////    int y_idx;
////    float resxy = m_kCartsGridmapParams.res_xy;
////    cv::Mat remain_points = cv::Mat::zeros(0,3,CV_32F);
////    for(int i=0; i<obstacle_points.rows; ++i)
////    {
////        move_x = obstacle_points.at<float>(i,0) - m_kCartsGridmapParams.min_x;
////        move_y = obstacle_points.at<float>(i,1) - m_kCartsGridmapParams.min_y;
////        x_idx = int(std::round(move_x / resxy));
////        y_idx = int(std::round(move_y / resxy));
////        if(x_idx >= 0 && x_idx < m_gridmap_h && y_idx >= 0 && y_idx < m_gridmap_w)
////        {
////            if(track_ins_map.at<uchar>(x_idx,y_idx) == 0)
////            {
////                remain_points.push_back(obstacle_points.row(i));
////            }
////        }
////    }

////    return remain_points;
////}
