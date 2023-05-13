
#include "rcs.hh"
#include "iostream"
#include "string"

#include "run.h"

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include <iomanip>

//#include "Commonfig.hh"
int main(int argc, char *argv[]) {

  std::cout<<"argc::"<<argv[0]<<std::endl;
  nml_start();



//  std::cout << "----------------------------------------" << std::endl;
//  std::cout << "Test" << std::endl;
////  ifstream ifs("/home/whb/Downloads/data/testrun/map/keyframe/keypose.pcd",ios::in);
////  ifstream ifs("/home/whb/Downloads/data/testrun/map/keyframe/0.pcd",ios::in);
//  std::string keypose_floder_ = "/home/whb/Downloads/data/testrun/map/keyframe";
//  pcl::PointCloud<PointType6D>::Ptr key_pose_;
//  key_pose_.reset(new pcl::PointCloud<PointType6D>);
//
//  std::cout<<"pcl::"<<keypose_floder_+"/keypose.pcd"<<std::endl;
//  if (pcl::io::loadPCDFile<PointType6D> (keypose_floder_+"/keypose.pcd", *key_pose_) == -1){
//    PCL_ERROR ("Couldn't read PCD file \n");
//    return 0;
//  }
////  for(int i =0;i<key_pose_->size();i++){
////    std::cout<<std::fixed<<"xy::"<<key_pose_->at(i).x<<","<<key_pose_->at(i).y<<","
////    <<key_pose_->at(i).time<<std::endl;
////  }
//    std::cout<<std::fixed<<"xy::"<<key_pose_->at(0).x<<","<<key_pose_->at(0).y<<","
//    <<key_pose_->at(0).time<<key_pose_->at(0).yaw<<std::endl;



  std::cout << "----------------------------------------" << std::endl;
  std::cout << "RUN" << std::endl;
  auto test_run = new TestRun();

//  test_run->folder_offline_ = argv[1];
  test_run->folder_offline_ = "/home/whb/Downloads/data/testrun/split";
//  test_run->folder_offline_ = "/media/gzdc-server2/cb11ced6-805e-4012-83ad-918d3cda419e/2023/Test/ltygs1/s1";
  test_run->run();

  nml_cleanup();
  return 0;

}


















