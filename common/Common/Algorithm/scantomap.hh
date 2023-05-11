#ifndef SCAN_TO_MAP_HH_
#define SCAN_TO_MAP_HH_
#include "isamOptimizer.hh"
#include "Common/MapDataFrame/MapDataStruct.hh"
#include "Common/ParamServer.hh"
#include "Common/Commonfig.hh"
#include "mutex"
//#include "backend.hh"
#include "Common/Algorithm/registrations.hpp"
//#include "Common/Algorithm/backend.hh"
#include <pcl/visualization/pcl_visualizer.h>
#include <qdialog.h>
#include <queue>
#include "Common/Algorithm/balmclass.hpp"
#include "gtsam/slam/OrientedPlane3Factor.h"
#include "match_core/ndt/match_score.hh"
#define USE_BATCH_OPTIMIZER 1
#if USE_BATCH_OPTIMIZER
//#include "batchG2oOptimizer.hh"
//class BATCHOPTIMIZER;
#endif

#define IMU_MAG_GAIN 100.0
#define deg_to_rad 0.01745329252
#define rad_to_deg  (1.0/deg_to_rad)
#define GRAVITY 9.80665


inline int globalMapVisualizationSearchRadius =50;

struct PlaneNew
{
    gtsam::OrientedPlane3 coffes;
    Eigen::Vector4f coffes_eigen;
    bool use_plane;
};


struct pose_graph
{
    int id1;
    int id2;
//    RPYpose node1;
//    RPYpose node2;
    string type;
};

class ISAMOPTIMIZER;
class SCANTOMAP
{
//     Q_OBJECT
public:
    SCANTOMAP(MAPDATAFRAME* MapDataFrame/*,BACKEND* BackEnd*/);
    ~SCANTOMAP();
    bool Process(HDLADARDATA_MSG* lidar_data);
    void ExtractSurroundKeyframes();
    void GetInsAndLpInfo();
    void GetLidarCloudAndFilter();
    void FrontEndThread();
    void ScanVoxelFilterDynamic();
    void Matching();
    bool KeyFrameUpdater();
    bool SaveFrame();
    void PerformLoopClosure();
    bool DetectLoopClosureDistance(int *latestID, int *closestID);
    void LoopFindNearKeyframes(pcl::PointCloud<pcl::PointXYZI>::Ptr& nearKeyframes, const int& key,const int& losest_key, const int& searchNum);

    void AllocateMemory();
    void ResetSystem();
    void GlobalBatchOptimizer();
    void Run();
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeHistoryKeyPoses;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeGlobalMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeNearLoopMap;
    pcl::PointCloud<PointT>::Ptr globalMapKeyPoses;
    pcl::PointCloud<PointT>::Ptr globalMapKeyPosesDS;
    pcl::PointCloud<PointT>::Ptr globalMapKeyFrames;
    pcl::PointCloud<PointT>::Ptr globalMapKeyFramesDS;
    pcl::PointCloud<PointT>::Ptr global2dMapKeyFrames;
    pcl::PointCloud<PointT>::Ptr global2dMapKeyFramesDS;
    pcl::PointCloud<PointT>::Ptr global2dMapKeyPoses;
    pcl::PointCloud<PointT>::Ptr global2dMapKeyPosesDS;
    pcl::VoxelGrid<PointT> downSizeFilterGlobalMapKeyPoses;
    pcl::VoxelGrid<PointT> downSizeFilterGlobalMapKeyFrames;
    pcl::PointCloud<PointT>::Ptr input_lidar,plane_lidar; // output cc show

    map<int, int> loop_IndexContainer; // from new to old

    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
    RPYpose _previous_pose, _current_pose, _added_pose,_pre_keypose,_first_pose, _gps_pose,
    _previous_gps,_current_gps,offsetlocalpose,localizer_pose,guess_pose;

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr,filter_scan_ptr,trans_filter_ptr;
    HDLADARDATA_MSG* lidardata;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;

    Eigen::Matrix4f initguess_matrix,current_lp_matrix,previous_lp_matrix,t_localizer,final_matrix,delta_lp_matrix;

    pcl::PointCloud<pcl::PointXYZI>::Ptr _target_points;
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> recent_keyframes_;

    pcl::PointCloud<PointType3D>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointType6D>::Ptr copy_cloudKeyPoses6D;
     pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterICP;

    int gps_state;
    bool loop_closure_enabled_;
    bool bGetZoneInit;
    bool _initial_scan_loaded;
    bool _loop_closed;

    Eigen::MatrixXd pose_covariance;
    float pose_cov_x;
    float pose_cov_y;

    pcl::PointXYZI currentRobotPosPoint;
    pcl::PointXYZI previousRobotPosPoint;

    RPYpose pre_keypose,cur_keypose;

    vector<pair<int, int>> loop_Index_Queue;
    vector<RPYpose> loop_Pose_from_Queue;
    vector<RPYpose> loop_Pose_to_Queue;

    vector<gtsam::Pose3> loop_Pose_Queue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loop_Noise_Queue;


    std::deque<pcl::PointCloud<pcl::PointXYZI>> cache_cloud_keyframes;
    std::deque<pcl::PointCloud<pcl::PointXYZI>> cache_cloud_keyframes_raw;
    std::deque<RPYpose> cache_ndt_pose;
    std::deque<PlaneNew> cache_plane;
    int plane_count;

    void AddOdomFactor();
    void AddGpsFactor(const RPYpose gnss_pose);
    void AddLoopFactor();
    void AddImuFactor();
    void CorrectPose();

    void HumanCorrectPose();

    gtsam::Pose3 pclPointTogtsamPose3(PointType6D thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    std::mutex mtx;
    std::mutex show_lck;
    std::mutex show_lidar_lck;

    boost::thread threadfrontEnd;
    boost::thread threadbackEnd;
    boost::thread threadloopdetect;
    boost::thread threadglobalMap;
    boost::thread thread3dMap;
    boost::thread threadBundleAdustmentOdom;

    void Visualize3DMapThread();
    void VisualizeGlobalMapThread();
    void LoopClosureThread();
    void BundleAdustmentOdomThread();
    void BackEndThread();
    void Publish3dMap();
    void Update3DshowMap();
    void publishGlobalMap();

    void add_lines_to_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                                 pcl::PointCloud<PointT>::Ptr pts_1, pcl::PointCloud<PointT>::Ptr pts_2,
                                                const std::string lines_name,
                                                const std::vector<unsigned int> &red, const std::vector<unsigned int> &green, const std::vector<unsigned int> &blue,
                                                int down_rate);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> pc_viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> InitViewer();

    bool bReset ;
    double time_LaserInfoCur;

    Eigen::Affine3f pclPointToAffine3f(PointType6D thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    float pointDistance(pcl::PointXYZI p1, pcl::PointXYZI p2)
    {
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }

    boost::shared_ptr< gtsam::NonlinearFactorGraph > gtSAMgraph_;
    boost::shared_ptr< gtsam::Values> initial_estimate_;
    boost::shared_ptr< gtsam::ISAM2 > isam;
    gtsam::Values isam_current_estimate_;


    double height_last;
    double height_history;
    double diff_height;

    vector<pose_graph> cons;

    void SelectPoints();
    int scanStartInd[65*20];   // 64+1,
    int scanEndInd[65*20];

    float cloudCurvature[PACKETNUM * 6 * 64 * 5];
    int cloudSortInd[PACKETNUM * 6 * 64 * 5];
    int cloudNeighborPicked[PACKETNUM * 6 * 64 * 5];
    int cloudLabel[PACKETNUM * 6 * 64 * 5];

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp;  // feature corn
    pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat;  // feature surf
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat;

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;
    bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

    mutex mBuf;
    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> surf_buf, corn_buf, full_buf;
    void GetCutVoxelMap(unordered_map<VOXEL_LOC, OCTO_TREE*> &feat_map, pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_feat, Eigen::Matrix3d R_p, Eigen::Vector3d t_p, int feattype, int fnum, int capacity);

    void BundleAdustmentOdom();
    bool PlaneDetection(const pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Vector4f& plane);
    pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative);
    pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud);

    Eigen::Vector4f coffes;
    unordered_map<VOXEL_LOC, OCTO_TREE*> surf_map, corn_map;
    bool bBA_init;

    LM_SLWD_VOXEL* opt_lsv;
    int window_size;
    int filter_num;
    int thread_num;
    int margi_size;
    int scan2map_on;

    int accumulate_window ;
    double surf_filter_length ;
    double corn_filter_length ;
    int plcount;
    int window_base;

    vector<Eigen::Quaterniond> delta_q_buf, q_buf;
    vector<Eigen::Vector3d> delta_t_buf, t_buf;

    pcl::PointCloud<PointType> pl_surf_centor_map, pl_corn_centor_map;
    pcl::PointCloud<PointType> pl_corn_fil_map, pl_surf_fil_map;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_corn;

    pcl::PointCloud<PointType> pl_send;
    Eigen::Vector3d p_orig, aft_tran, kervec, orient, v_ac;

    uint a_size;
    PointType apy;
    vector<int> pointSearchInd; vector<float> pointSearchSqDis;
    double range;
    Eigen::Matrix4d trans;
    int pub_skip;

    pcl::PointCloud<PointType>::Ptr pl_corn;
    pcl::PointCloud<PointType>::Ptr pl_surf;
    vector<pcl::PointCloud<PointType>::Ptr> pl_full_buf;

    void BAInitial();
    void PointXYZI2PointType(pcl::PointCloud<pcl::PointXYZI> &pl, pcl::PointCloud<PointType> &plt);

    double tilt_deg;
    double sensor_height;
    double height_clip_range;

    int floor_pts_thresh;
    double floor_normal_thresh;

    bool use_normal_filtering;
    double normal_filter_thresh;

    void update_fitness_score(const Eigen::Matrix4f& relpose);

    double calc_fitness_score(const pcl::PointCloud<PointT>::Ptr& cloud1, const pcl::PointCloud<PointT>::Ptr& cloud2,
                                        const Eigen::Matrix4f& relpose, double max_range);

    void processMatchingScore(pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr);

    void getMatchAndUnmatchPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr match_points_ptr,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_points_ptr);

    void updateMatchingScore(pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr);

    //keyframe data
//    pcl::PointCloud<PointType3D>::Ptr keyposes_3d_ptr;
//    pcl::PointCloud<PointType2D>::Ptr keyposes_2d_ptr;
//    pcl::PointCloud<PointType6D>::Ptr keyposes_6d_ptr;

    MAPDATAFRAME* mMapDataFrame;
    float time;
    int keyNum;
    int lidar_raw_size;
    int lidar_filter_size;
    float lon_dev;
    float lat_dev;
    float covx;
    float covy;
    float covz;
    float lp_yaw;
    double lp_time;
    int map_init_state;
    int gp_node;
    double init_yaw;

    std::deque<HDLADARDATA_MSG*> lidar_buffer_;
    std::deque<LOCALPOSE_MSG*> imu_buffer_;
    std::deque<double> time_buffer_;
    void LidarCallBack(HDLADARDATA_MSG* msg);
    void IMUCallBack(LOCALPOSE_MSG* msg);
    std::mutex mutex_buffer_;

    double last_timestamp_imu_ = -1;
    double last_timestamp_lidar_ = 0;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;


    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;

    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    double lastImuT_opt;

    float imuAccNoise = 0.009939570888238808e-03;
    float imuGyrNoise = 0.005636343949698187e-03;
    float imuAccBiasN = 0.64356659353532566e-03;
    float imuGyrBiasN = 0.35640318696367613e-03;

private:
    double match_score_range;
    double fitness_score_loop;
    float history_fitness_score_;
    float match_score;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cureKeyframeCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr prevKeyframeCloud;
    MatchingScore* matchingscore;
    int save_count;

protected:

//    BACKEND* mBackEnd;


};


#endif//SCAN_TO_MAP_HH_
