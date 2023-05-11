#include "3DMatch2D.h"

//栅格地图hit和miss的概率
float hit_value_ = 0.9;
float miss_value_ = 0.1;
std::vector<uint16> hit_table;
std::vector<uint16> miss_table;

P_map create_Map(std::vector<KeyFrame> keyframes){
//    // 校正位姿
//    int middle_index = keyframes.size() / 2;
//    auto middle_pose = keyframes[middle_index].pose;
//    for(int i = 0; i < keyframes.size(); i++){
//        keyframes[i].pose = middle_pose.inverse() * keyframes[i].pose;
//    }

    //初始化两个table
    hit_table = ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(hit_value_));
    miss_table = ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(miss_value_));

    P_map p_map;
    Lidar_range map_range(Eigen::Vector3f::Identity(), Eigen::Vector3f::Zero());

    MAPPER_INFO("creating a pmap ...");
    p_map.p_map_resolution = 0.05;

    //先计算栅格地图的大小
    Eigen::AlignedBox2f bounding_box;
    for (auto kf : keyframes) {
        bounding_box.extend(kf.pose.head<2>());
    }

    Eigen::Vector3f min_point = Eigen::Vector3f(bounding_box.min().x() - 40.,
                                                bounding_box.min().y() - 40.,
                                                0.f);
    Eigen::Vector3f max_point = Eigen::Vector3f(bounding_box.max().x() + 40.,
                                                bounding_box.max().y() + 40.,
                                                0.f);
    map_range.min_xy = min_point;
    map_range.max_xy = max_point;
    map_range.calculate_vertex();

    p_map.ResetByRange(map_range);

    for (auto kf : keyframes)
    {
        PCLPoints cloud = kf.point;
        Eigen::Matrix4f kf_pose = array_to_matrix(std::array<float, 3>{kf.pose[0],
                                                                       kf.pose[1],
                                                                       kf.pose[2]});
        pcl::transformPointCloud(cloud, cloud, kf_pose);

        Point_cloud2d inserted_scan = PclToPointcloud2d(cloud, kf_pose);
        p_map.addscan(inserted_scan, hit_table, miss_table);
    }
    MAPPER_INFO("create a pmap down.");

    return p_map;
}

map_t* conver_map(P_map& probability_map){
    map_t *map = map_alloc();
    map->size_x = probability_map.p_map_width;
    map->size_y = probability_map.p_map_height;
    map->scale = probability_map.p_map_resolution;
//    map->origin_x = 0.f + (map->size_x / 2) * map->scale;
//    map->origin_y = 0.f + (map->size_y / 2) * map->scale;
    map->origin_x = probability_map.p_map_origin_x;
    map->origin_y = probability_map.p_map_origin_y;

    map->cells = (map_cell_t *)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

    for(size_t i = 0; i < probability_map.p_map_cells.size(); i++){
        auto value = probability_map.p_map_cells[i];
        if(value.visit == -1){
            //unknow
            map->cells[i].occ_state = 0;
            continue;
        }
        float cell_p = (float)value.hit_num/(float)value.visit;
        if(cell_p > 0.1){
            //occ
            map->cells[i].occ_state = +1;
        }
        else{
            //free
            map->cells[i].occ_state = -1;
        }
    }

    // 更新地图以便计算得分
    map_update_cspace(map, 2.0);

    return map;
}


bool gloabMapCost(map_t *map, Eigen::Vector2d cur_point, double &z)
{
    int mi = MAP_GXWX(map, cur_point[0]);
    int mj = MAP_GYWY(map, cur_point[1]);
    if (!MAP_VALID(map, mi, mj))
    {
        z = 1;
        return false;
    }
    else
    {
        z = map->cells[MAP_INDEX(map, mi, mj)].occ_dist / map->max_occ_dist;
        return true;
    }
}

Eigen::Vector3d real_time_csm(PCLPoints lidar, Eigen::Vector3d mean, map_t *map,
                              double start_depth, float &best_score_)
{
    auto t1 = std::chrono::steady_clock::now();
    Eigen::Vector3d searchResolution(0.05, 0.05, 0.05);
    searchResolution = searchResolution * start_depth;

    std::vector<Eigen::Vector2d> laser_points;
    for (size_t i = 0; i < lidar.points.size(); i++)
    {
        Eigen::Vector2d point(lidar.points[i].x, lidar.points[i].y);
        laser_points.push_back(point);
    }

    auto best_pose = mean;
    double baset_score = -1;
    //
    int depth = 0;
    int maxDepth = 4;
    int iter = 0;
    int maxIter = 50;
    bool isDownsize = true;
    std::vector<std::pair<Eigen::Vector3f, float>> pose_score_pair;
    while (iter < maxIter)
    {
        isDownsize = true;
        for (auto angle : {-searchResolution[0], 0.0, searchResolution[0]})
        {
            double theta = angle + best_pose[2];
            Eigen::Rotation2Dd rot(theta);
            std::vector<Eigen::Vector2d> search_points;
            for (auto point : laser_points)
            {
                auto rot_point = rot * point;
                search_points.push_back(rot_point);
            }
            for (auto x : {-searchResolution[1], 0.0, searchResolution[1]})
            {

                for (auto y : {-searchResolution[2], 0.0, searchResolution[2]})
                {
                    double score = 0;
//                    double local_score = 0;
                    Eigen::Vector2d xy(best_pose[0] + x, best_pose[1] + y);
                    //
                    double valid_point_size = 0;
//                    double local_valid_point_size = 0;
                    // for (auto point : search_points)
                    for (size_t n = 0; n < search_points.size(); n++)
                    {
                        Eigen::Vector2d point = search_points[n];
                        auto cur_point = point + xy;

                        double z = 0;
                        bool is_inliner = gloabMapCost(map, cur_point, z);

                        if (!is_inliner)
                        {
                            // continue;
                        }
                        else
                        {
                            valid_point_size++;
                            score += z;
                        }

                    }
                    score /= (valid_point_size + 0.1);
                    score = (1.0 - score);

//                    Eigen::Vector3d delta = Eigen::Vector3d(xy[0], xy[1], theta) - mean;
//                    score *= std::exp(-std::pow(std::hypot(delta[0], delta[1]) * 0.5 + std::abs(delta[2]) * 0.5, 2.));
                    if (score > baset_score)
                    {

                        baset_score = score;
                        best_pose = Eigen::Vector3d(xy[0], xy[1], theta);
                        isDownsize = false;
                        pose_score_pair.emplace_back(best_pose.cast<float>(), baset_score);
                    }
                }
            }
        }
        if (isDownsize)
        {
            searchResolution = searchResolution / 2.0;
            if (depth++ > maxDepth)
                break;
        }
        iter++;
    }

    auto t2 = std::chrono::steady_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    best_score_ = float(baset_score);
    return best_pose;
}

Posef match(PCLPoints& point, Posef init_pose, P_map& p_map){
    map_t* occ_map = conver_map(p_map);

    float score;
    Eigen::Vector3d mean = init_pose.cast<double>();
    Eigen::Vector3d match_pose = real_time_csm(point, mean, occ_map, 1.0, score);

    return match_pose.cast<float>();
}
