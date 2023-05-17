//
// Created by whb on 2023/4/19.
//

#ifndef TEST_RUN_RUN_H
#define TEST_RUN_RUN_H

#include <vector>
#include <string>
#include <dirent.h>
#include <cstring>
#include <algorithm>
#include <opencv2/core/mat.hpp>
#include <omp.h>
#include <iostream>
#include <memory>

#include "data_analysis.h"

class TestRun {
public:
  TestRun();

  ~TestRun();

  void run();

  void MakeNML(long num);

private:
  void initialize();

public:
    std::string folder_offline_{};
    std::string map_path_{};

private:
  std::vector<std::string> folders_offline_{}; //分包后数据

};


#endif //TEST_RUN_RUN_H
