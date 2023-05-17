
#include <wait.h>
#include <QThread>
#include <iostream>
#include <regex>
#include "run.h"

TestRun::TestRun() = default;

TestRun::~TestRun() = default;

void TestRun::initialize() {
  std::string str_temp;
  DIR *pDir;
  pDir = opendir(folder_offline_.c_str());
  struct dirent *ptr;
  while ((ptr = readdir(pDir)) != nullptr) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      str_temp = ptr->d_name;
      if (str_temp != "result") {
        str_temp = folder_offline_ + "/" + ptr->d_name;
        folders_offline_.emplace_back(str_temp);
//        folders_offline_clear_.emplace_back(ptr->d_name);
      }
    }
  }
  closedir(pDir);
  std::sort(folders_offline_.begin(), folders_offline_.end());
//  std::sort(folders_offline_clear_.begin(), folders_offline_clear_.end());
//  MakeNML(folders_offline_.size());
//  MakeNML(100);
}

void TestRun::run() {
  initialize();

  char *ugv_ptr = std::getenv("LD_LIBRARY_PATH");
  if (ugv_ptr == nullptr) {
    std::cerr << "Could not get env LD_LIBRARY_PATH." << std::endl;
    std::exit(EXIT_FAILURE);
  } else {
    std::cout << "LD_LIBRARY_PATH" << ugv_ptr << std::endl;
  }

  std::string rr0{};
  std::cout << "运行 mv lib to x86 " << std::endl;
  rr0 = "mv " + std::string(ugv_ptr) + "/libgtsam/libgtsam.so.4 " + std::string(ugv_ptr);
  system(rr0.c_str());
  rr0 = "mv " + std::string(ugv_ptr) + "/libgtsam/libgtsam_unstable.so.4 " + std::string(ugv_ptr);
  system(rr0.c_str());
  rr0 = "mv " + std::string(ugv_ptr) + "/libgtsam/libmetis-gtsam.so " + std::string(ugv_ptr);
  system(rr0.c_str());
  std::cout << "now::x86/libgtsam.so.4" << "x86/libgtsam_unstable.so.4" << "x86/libmetis-gtsam.so" << std::endl;

  struct stat buffer{};
  string f_name = std::string(ugv_ptr) + "/libgtsam.so.4";
  struct stat buffer1{};
  string f_name1 = std::string(ugv_ptr) + "/libgtsam_unstable.so.4";
  struct stat buffer2{};
  string f_name2 = std::string(ugv_ptr) + "/libmetis-gtsam.so";
  while (stat(f_name.c_str(),&buffer)!=0||stat(f_name1.c_str(), &buffer1)!=0||stat(f_name2.c_str(), &buffer2)!=0){
    std::cout<<"no::"<<stat(f_name.c_str(),&buffer)<<stat(f_name1.c_str(), &buffer1)
             <<stat(f_name2.c_str(), &buffer2)<<std::endl;
    sleep(1);
  }

  for (int i = 0; i < folders_offline_.size() * 4; i++) {
    int status{};

    pid_t child_pid = fork();
    if (child_pid == -1) {
      std::cout << "error::" << std::endl;
    } else if (child_pid > 0) {
      std::cout << "开始运行子进程：" << int(child_pid) << std::endl;
      if(i == folders_offline_.size()-1){

        FILE * fp{} ;
        char returnData[1024];
        int count_FusionLocalizer{};
        fp= popen("ps gaux|grep FusionLocalizer |grep -v grep |wc -l","r");
        fgets(returnData, 1024, fp);
        count_FusionLocalizer = stoi(returnData);
        std::cout<<"count_FusionLocalizer::"<<count_FusionLocalizer<<std::endl;
        while(count_FusionLocalizer != folders_offline_.size()*2){
          sleep(1);
        }

        rr0 = "mv " + std::string(ugv_ptr) + "/libgtsam.so.4 " + std::string(ugv_ptr) + "/libgtsam";
        std::cout<<"命令："<<rr0<<std::endl;
        system(rr0.c_str());
        rr0 = "mv " + std::string(ugv_ptr) + "/libgtsam_unstable.so.4 " + std::string(ugv_ptr) + "/libgtsam";
        std::cout<<"命令："<<rr0<<std::endl;
        system(rr0.c_str());
        rr0 = "mv " + std::string(ugv_ptr) + "/libmetis-gtsam.so " + std::string(ugv_ptr) + "/libgtsam";
        std::cout<<"命令："<<rr0<<std::endl;
        system(rr0.c_str());
        std::cout << "now::lib/libgtsam.so.4" << "lib/libgtsam_unstable.so.4" << "lib/libmetis-gtsam.so" << std::endl;


//        struct stat buffer{};
        f_name = std::string(ugv_ptr) + "/libgtsam/libgtsam.so.4";
//        struct stat buffer1{};
        f_name1 = std::string(ugv_ptr) + "/libgtsam/libgtsam_unstable.so.4";
//        struct stat buffer2{};
        f_name2 = std::string(ugv_ptr) + "/libgtsam/libmetis-gtsam.so";
        while (stat(f_name.c_str(),&buffer)!=0||stat(f_name1.c_str(), &buffer1)!=0||stat(f_name2.c_str(), &buffer2)!=0){
          std::cout<<"no::"<<stat(f_name.c_str(),&buffer)<<stat(f_name1.c_str(), &buffer1)
          <<stat(f_name2.c_str(), &buffer2)<<std::endl;
          sleep(1);
        }
      }
      if(i == folders_offline_.size()*2-1){
        int count_M{};
        FILE * fp{} ;
        char returnData[1024];
        do{
          sleep(1);
          fp= popen("ps gaux|grep MapLocalizer_x86 |grep -v grep |wc -l","r");
          fgets(returnData, 1024, fp);
          count_M = stoi(returnData);
          std::cout<<"count_TestRun::"<<count_M<<std::endl;
        } while (count_M != folders_offline_.size()*2);
        fclose(fp);
      }

    } else if (child_pid == 0) {
      if (i < folders_offline_.size()) {
        std::string rr1;
        std::cout << "运行 FusionLocalizer " << i << std::endl;
        rr1 = "./FusionLocalizer " + std::to_string(i);
        system(rr1.c_str());

//        int t =181;
//        while (t < 200&&t>180) {
//          QThread::sleep(1);
//          stringstream ss;
//          ss << setw(3) << setfill('0') << t ;
//          std::cout<<"FusionLocalizer::"<< ss.str() <<std::endl;
//          t++;
//        }

        std::cout << "子进程" << getpid() << "退出了" << std::endl;
        _exit(0);
      }
      if (i >= folders_offline_.size() && i < folders_offline_.size() * 2) {
        std::string rr;
        std::cout << "运行 MapLocalizer_x86 " << i << std::endl;
        rr = "./MapLocalizer_x86 -p " + std::to_string(i - folders_offline_.size());
        system(rr.c_str());

//        int t =281;
//        while (t < 300&&t>280) {
//          QThread::sleep(1);
//          stringstream ss;
//          ss << setw(3) << setfill('0') << t ;
//          std::cout<<"MapLocalizer_x86::"<< ss.str() <<std::endl;
//          t++;
//        }

        std::cout << "子进程" << getpid() << "退出了" << std::endl;
        _exit(0);
      }
      if (i >= folders_offline_.size() * 2 && i < folders_offline_.size() * 3) {
        std::string rr;
        std::cout << "运行 FusionLocScore_rcs " << i << std::endl;
        rr = "./FusionLocScore_rcs -p " + std::to_string(i - folders_offline_.size() * 2);
        system(rr.c_str());

//        int t =381;
//        while (t < 400&&t>380) {
//          QThread::sleep(1);
//          stringstream ss;
//          ss << setw(3) << setfill('0') << t ;
//          std::cout<<"FusionLocScore_rcs::"<< ss.str() <<std::endl;
//          t++;
//        }

        std::cout << "子进程" << getpid() << "退出了" << std::endl;
        _exit(0);
      }

      if (i >= folders_offline_.size() * 3 && i < folders_offline_.size() * 4) {

        auto data = std::make_shared<DataAnalysis>(folders_offline_.at(i - folders_offline_.size() * 3),i - folders_offline_.size() * 3,map_path_);
        data->Analysis();

//        int t =0;
//        while (t < 20) {
//          QThread::sleep(1);
//          stringstream ss;
//          ss << setw(3) << setfill('0') << t ;
//          std::cout<<"Analysis::"<< ss.str() <<std::endl;
//          t++;
//        }
        std::cout << "子进程" << getpid() << "退出了" << std::endl;
        _exit(0);
      }
    }

    // TODO
    // 将TestRun改为单独传参程序，由新版无人值守调用,

    if(i == folders_offline_.size()*4-1){
      sleep(10);

      int count_TestRun{};
      FILE * fp{} ;
      char returnData[1024];
      do{
        sleep(1);
        fp= popen("ps gaux|grep TestRun |grep -v grep |wc -l","r");
        fgets(returnData, 1024, fp);
        count_TestRun = stoi(returnData);
        std::cout<<"count_TestRun::"<<count_TestRun<<std::endl;
      } while (count_TestRun != folders_offline_.size() * 4);

      FILE * fp1{} ;
      char returnData1[1024];
      std::string s;
      fp1= popen("tty","r");
      fgets(returnData1, 1024, fp1);
      s = returnData1;
      std::cout<<"s::"<<s<<std::endl;
      std::string ssr;
      ssr = s.substr(5);
      std::cout<<"ssr::"<<ssr<<std::endl;

      std::string rr;
      std::cout << "运行 pkill " << std::endl;
      rr = "pkill -f FusionLocalizer -e -t " + ssr;
      system(rr.c_str());
      rr = "pkill -f MapLocalizer_x86 -e -t " + ssr;
      system(rr.c_str());
      rr = "pkill -f FusionLocScore_rcs -e -t " + ssr;
      system(rr.c_str());

      fclose(fp1);
      std::cout << "end1!!" << std::endl;
    }
  }
  std::cout << "Done all!" << std::endl;
}

void TestRun::MakeNML(long num) {

  std::fstream fs_file;
  fs_file.open("UGVAuto.nml", std::ios::out | std::ios::trunc);

  std::vector<std::string> b_name{"GlobalPositionBuffer", "LocalPose", "LadarMiddleMiddleTop", "MapPosition",
                                  "MapPositionInit", "LidarPosition", "FusionPositionScore"};
  std::vector<std::string> b_size{"20480", "10240", "1280000", "20480", "20480", "20480", "20480"};
  std::vector<std::string> b_name_m{"LadarFloatMiddleMiddleTop", "LadarRightFrontTop", "LadarMiddleFrontBottom",
                                    "SyncLidarData", "MapperState", "TaskList", "GlobalPositionBuffer", "LocalPose",
                                    "MapLocalizer3DControlRequest", "MapLocalizer3DControlResponse", "LocalHDMap"};
  // ? 没有声明，我就去除了 "PathnetPcdName", "SDKMapPosition","MapPointCloud"
  std::vector<std::string> b_size_m{"2560000", "1280000", "1280000", "5120000", "10240", "204800", "20480", "10240",
                                    "5120", "5120", "5000000"};
  std::vector<std::string> p_name{"TransformFusion", "TestRun", "FusionLocScore", "MapLocalizer"};

  fs_file << "# Buffers\n";
  int b_num{};
  for (int i = 0; i < b_name.size(); ++i) {
    fs_file << "# " << b_name.at(i) << "\n";
    for (int j = 0; j < num; ++j) {
      stringstream ss;
      ss << setw(3) << setfill('0') << j + 1 + i * num;
      b_num = j + 1 + i * num;
      std::string tmp =
          "B " + b_name.at(i) + to_string(j) + "  SHMEM  localhost  " + b_size.at(i) + "  0  *  " + ss.str()
          + "  *  1" + ss.str() + "  bsem=11" + ss.str() + "  TCP=11" + ss.str() + "  xdr\n";
      fs_file << tmp;
    }
  }
  fs_file << "# other Buffers\n";
  for (int i = 0; i < b_name_m.size(); ++i) {
    stringstream ss;
    ss << setw(3) << setfill('0') << b_num + 1 + i;
    std::string tmp = "B " + b_name_m.at(i) + "  SHMEM  localhost  " + b_size_m.at(i) + "  0  *  " + ss.str()
                      + "  *  1" + ss.str() + "  bsem=11" + ss.str() + "  TCP=11" + ss.str() + "  xdr\n";
    fs_file << tmp;
  }
  fs_file << "\n# Processes\n";
  for (int k = 0; k < p_name.size(); ++k) {
    for (int i = 0; i < num; ++i) {
      fs_file << "# " << p_name.at(k) << i << "\n";
      for (int j = 0; j < b_name.size(); ++j) {
        stringstream ss;
        ss << i + k * num + 100;
        std::string tmp = "P " + p_name.at(k) + to_string(i) + "  " + b_name.at(j) + to_string(i)
                          + " LOCAL localhost RW  0 0.1 1 " + ss.str() + "\n";
        fs_file << tmp;
      }
      for (int j = 0; j < b_name_m.size(); ++j) {
        stringstream ss;
        ss << i + k * num + 100;
        std::string tmp = "P " + p_name.at(k) + to_string(i) + "  " + b_name_m.at(j)
                          + " LOCAL localhost RW  0 0.1 1 " + ss.str() + "\n";
        fs_file << tmp;
      }
      fs_file << "\n";
    }
  }
  fs_file.close();
}
//              "B GlobalPositionBuffer  SHMEM  localhost  20480    0  *  001  *  1001  bsem=11001  TCP=11001  xdr\n"
//              "B LocalPose             SHMEM  localhost  10240    0  *  002  *  1002  bsem=11002  TCP=11002  xdr\n"
//              "B LadarMiddleMiddleTop  SHMEM  localhost  1280000  0  *  003  *  1003  bsem=11003  TCP=11003  xdr\n"
//              "B MapPosition           SHMEM  localhost  20480    0  *  100  *  1100  bsem=11100  TCP=11100  xdr\n"
//              "B MapPositionInit       SHMEM  localhost  20480    0  *  227  *  1227  bsem=11227  TCP=11227  xdr\n"
//              "B LidarPosition         SHMEM  localhost  20480    0  *  223  *  1223  bsem=11223  TCP=11223  xdr\n"
//              "B FusionPositionScore   SHMEM  localhost  20480    0  *  284  *  1284  bsem=11301  TCP=11301  xdr\n"