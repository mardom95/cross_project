// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Mario Domínguez López
 */

#include "mecanum_drive_controller/odometry.hpp"

namespace mecanum_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linearx_(0.0),
  lineary_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  left_wheel_radius_(0.0),
  right_wheel_radius_(0.0),
  fl_wheel_old_pos_(0.0),
  fr_wheel_old_pos_(0.0),
  rl_wheel_old_pos_(0.0),
  rr_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linearx_accumulator_(velocity_rolling_window_size),
  lineary_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update( double front_left_pos, double front_right_pos, double rear_left_pos, double rear_right_pos,  const rclcpp::Time &time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001) {
    return false;    // Interval too small to integrate with
  }

  const double front_left_vel = front_left_pos - fl_wheel_old_pos_;
  const double front_right_vel = front_right_pos - fr_wheel_old_pos_;
  const double rear_left_vel = rear_left_pos - rl_wheel_old_pos_;
  const double rear_right_vel = rear_right_pos - rr_wheel_old_pos_;

  /// linearx and y and increments in robot cartesian coordinates, position
  /// Compute linear and angular diff
  /// Assuming wheel separation as lx + ly
  const double linearx  = (left_wheel_radius_/4)*(+rear_right_vel + front_right_vel + rear_left_vel + front_left_vel);
  const double lineary  = (left_wheel_radius_/4)*(-rear_right_vel + front_right_vel + rear_left_vel - front_left_vel);
  const double angular = (left_wheel_radius_/(4*wheel_separation_))*(rear_right_vel + front_right_vel - rear_left_vel - front_left_vel);

  //! Update old values
  fl_wheel_old_pos_ = front_left_pos;
  fr_wheel_old_pos_ = front_right_pos;
  rl_wheel_old_pos_ = rear_left_pos;
  rr_wheel_old_pos_ = rear_right_pos;

  /// Integrate odometry:
  integrateExact(linearx*dt, lineary*dt, angular_*dt);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linearx_accumulator_.accumulate(linearx / dt);
  lineary_accumulator_.accumulate(lineary / dt);
  angular_accumulator_.accumulate(angular / dt);

  linearx_ = linearx_accumulator_.getRollingMean();
  lineary_ = lineary_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

//! DEPRECATED NOT USE
void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linearx_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt, linear);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation, double left_wheel_radius,
  double right_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double dx, double dy, double da)
{

  heading_ += da;

  double direction = std::atan2(dy,dx);
  double m = std::sqrt(std::pow(dx,2) + std::pow(dy,2));

  x_       += m*cos(direction + heading_);
  y_       += m*sin(direction + heading_);

}

void Odometry::resetAccumulators()
{
  linearx_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  lineary_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace diff_drive_controller
