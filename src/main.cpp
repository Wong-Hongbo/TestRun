
#include "rcs.hh"
#include "iostream"
#include "string"

#include "run.h"

int main(int argc, char *argv[]) {

  std::cout<<"argc::"<<argv[0]<<std::endl;
  nml_start();

  std::cout << "----------------------------------------" << std::endl;
  std::cout << "RUN" << std::endl;
  auto test_run = new TestRun();

//  test_run->folder_offline_ = argv[1];
  test_run->folder_offline_ = "/home/whb/Downloads/data/testrun/ltygs1";
//  test_run->folder_offline_ = "/media/gzdc-server2/cb11ced6-805e-4012-83ad-918d3cda419e/2023/Test/ltygs1/s1";
  test_run->run();

  nml_cleanup();
  return 0;

}


















