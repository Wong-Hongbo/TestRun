
#include "rcs.hh"
#include "iostream"
#include "string"
#include "version.h"
#include "run.h"

// test
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include <regex>

int main(int argc, char *argv[]) {
  std::cout << "----------------------------------------" << std::endl;
  std::cout << "Version" << std::endl;
  int opt = -1;
  while ((opt = getopt(argc, argv, "vha:m:")) != -1) {
    switch (opt) {
      case 'v':
        printf("%s %s %s %s released.\n", PROJECT_NAME, PROJECT_VERSION, __DATE__, __TIME__);
        return 0;
      case 'h':
        printf("Usage: %s [-v] [-h] [-a para] [-m para]\n", argv[0]);
        return 0;
      case 'a':
        printf("option [a] param: %s\n", optarg);
        break;
      case 'm':
        printf("option [m] param: %s\n", optarg);
        break;
      default: /* '?' */
        break;
    }
  }

  std::cout<<"argc::"<<argv[0]<<std::endl;
  nml_start();

//  std::cout << "----------------------------------------" << std::endl;
//  std::cout << "Test" << std::endl;

  std::cout << "----------------------------------------" << std::endl;
  std::cout << "RUN" << std::endl;
  auto test_run = new TestRun();

//  test_run->folder_offline_ = argv[1];
//  test_run->folder_offline_ = "/media/gzdc-server2/cb11ced6-805e-4012-83ad-918d3cda419e/2023/Test/ltygs1/s1";

  test_run->folder_offline_ = "/home/whb/Downloads/data/testrun/split";
  test_run->map_path_ = "/home/whb/Downloads/data/testrun/map/keyframe";
  test_run->run();

  nml_cleanup();
  return 0;

}


















