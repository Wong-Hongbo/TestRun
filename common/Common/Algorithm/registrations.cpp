// SPDX-License-Identifier: BSD-2-Clause

#include "registrations.hpp"
#include <iostream>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include "Common/Algorithm/match_core/ndt/ndt_omp.h"
#include "Common/Algorithm/match_core/ndt/gicp_omp.h"
#include "fast_gicp/gicp/fast_gicp.hpp"
#include "fast_gicp/gicp/fast_vgicp.hpp"
#include "Common/ParamServer.hh"
#ifdef USE_VGICP_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> select_registration_method() {
  using PointT = pcl::PointXYZI;
    string registration_method;
      // select a registration method (ICP, GICP, NDT)
    std::cout << "odom_type : " << odom_type << std::endl;
    if(odom_type==0)
    {
        std::cout << "FAST_GICP" << std::endl;
        registration_method = "FAST_GICP";
    }
    else if(odom_type==1)
    {
        std::cout << "FAST_VGICP" << std::endl;
        registration_method = "FAST_VGICP";
    }
    else if(odom_type==2)
    {
        std::cout << "ICP" << std::endl;
        registration_method = "ICP";
    }
    else if(odom_type==3)
    {
        std::cout << "GICP_OMP" << std::endl;
        registration_method = "GICP_OMP";
    }
    else if(odom_type==4)
    {
        std::cout << "NDT_OMP" << std::endl;
        registration_method = "NDT_OMP";
    }

  if(registration_method == "FAST_GICP") {
    std::cout << "registration: FAST_GICP" << std::endl;
    boost::shared_ptr<fast_gicp::FastGICP<PointT, PointT>> gicp(new fast_gicp::FastGICP<PointT, PointT>());
    gicp->setNumThreads(0);
    gicp->setTransformationEpsilon(0.01);
    gicp->setMaximumIterations(64);
    gicp->setMaxCorrespondenceDistance(2.5);
    gicp->setCorrespondenceRandomness(20);
    return gicp;
  }
#ifdef USE_VGICP_CUDA
  else if(registration_method == "FAST_VGICP_CUDA") {
    std::cout << "registration: FAST_VGICP_CUDA" << std::endl;
    boost::shared_ptr<fast_gicp::FastVGICPCuda<PointT, PointT>> vgicp(new fast_gicp::FastVGICPCuda<PointT, PointT>());
    vgicp->setResolution(pnh.param<double>("reg_resolution", 1.0));
    vgicp->setTransformationEpsilon(pnh.param<double>("reg_transformation_epsilon", 0.01));
    vgicp->setMaximumIterations(pnh.param<int>("reg_maximum_iterations", 64));
    vgicp->setCorrespondenceRandomness(pnh.param<int>("reg_correspondence_randomness", 20));
    return vgicp;
  }
#endif
  else if(registration_method == "FAST_VGICP")
  {
    std::cout << "registration: FAST_VGICP" << std::endl;
    boost::shared_ptr<fast_gicp::FastVGICP<PointT, PointT>> vgicp(new fast_gicp::FastVGICP<PointT, PointT>());
    vgicp->setNumThreads(0);
    vgicp->setResolution(1.0);
    vgicp->setTransformationEpsilon(0.01);
    vgicp->setMaximumIterations(64);
    vgicp->setCorrespondenceRandomness(20);
    return vgicp;
  }
  else if(registration_method == "ICP")
  {
    std::cout << "registration: ICP" << std::endl;
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEpsilon(0.01);
    icp->setMaximumIterations(64);
    icp->setMaxCorrespondenceDistance(2.5);
    icp->setUseReciprocalCorrespondences(false);
    return icp;
  }
  else if(registration_method.find("GICP") != std::string::npos)
  {
    if(registration_method.find("OMP") == std::string::npos)
    {
      std::cout << "registration: GICP" << std::endl;
      boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(0.01);
      gicp->setMaximumIterations(64);
      gicp->setUseReciprocalCorrespondences(false);
      gicp->setMaxCorrespondenceDistance(2.5);
      gicp->setCorrespondenceRandomness(20);
      gicp->setMaximumOptimizerIterations(20);
      return gicp;
    } else {
      std::cout << "registration: GICP_OMP" << std::endl;
      boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(0.01);
      gicp->setMaximumIterations(64);
      gicp->setUseReciprocalCorrespondences(false);
      gicp->setMaxCorrespondenceDistance(2.5);
      gicp->setCorrespondenceRandomness(20);
      gicp->setMaximumOptimizerIterations(20);
      return gicp;
    }
  }
  else if(registration_method == "NDT_OMP")
  {
      double ndt_resolution = 1.0;
      int num_threads = 1;
      std::string nn_search_method = "DIRECT1";
      std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
      boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      if(num_threads > 0)
      {
        ndt->setNumThreads(num_threads);
      }
      ndt->setTransformationEpsilon(0.01);
      ndt->setMaximumIterations(64);
      ndt->setResolution(ndt_resolution);
      if(nn_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else if(nn_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      }
      return ndt;
  }

  return nullptr;
}

