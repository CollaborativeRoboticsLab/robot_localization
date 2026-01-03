#pragma once

#include "robot_localization/fault/ros_filter_ft.hpp"
#include "robot_localization/fault/ekf_ft.hpp"
#include "robot_localization/fault/ukf_ft.hpp"

namespace robot_localization
{
typedef RosFilterFT<UkfFT> RosUkfFT;
typedef RosFilterFT<EkfFT> RosEkfFT;
}