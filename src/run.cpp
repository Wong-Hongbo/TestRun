
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
}

void TestRun::run() {

  initialize();

#pragma omp parallel for
  for (int i = 0; i < 2; ++i) {
    auto data = std::make_shared<DataAnalysis>(folders_offline_.at(i));
    data->Analysis();
    std::cout << "Done : " << i << " !" << std::endl;
  }
  std::cout << "Done all!" << std::endl;
}

