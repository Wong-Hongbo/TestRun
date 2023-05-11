// SPDX-License-Identifier: BSD-2-Clause

#ifndef REGISTRATIONS_HPP
#define REGISTRATIONS_HPP

#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include "Common/Algorithm/match_core/ndt/ndt_omp.h"
#include "Common/Algorithm/match_core/ndt/gicp_omp.h"
#include "fast_gicp/gicp/fast_gicp.hpp"
#include "fast_gicp/gicp/fast_vgicp.hpp"


/**
 * @brief select a scan matching algorithm according to rosparams
 * @param pnh
 * @return selected scan matching
 */
boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> select_registration_method();


#endif  //
