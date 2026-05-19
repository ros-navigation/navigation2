// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// In-process singleton that shares the ESDF grid between ConstrainedController
// and EsdfCritic.  Both run inside the same controller_server process and on
// the same thread (ESDF is updated before mppi_->computeVelocityCommands is
// called, which synchronously invokes the critic), so contention is minimal.

#ifndef NAV2_CONSTRAINED_CONTROLLER__ESDF_REGISTRY_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__ESDF_REGISTRY_HPP_

#include <mutex>
#include <string>
#include <unordered_map>

namespace nav2_constrained_controller
{

class EsdfGrid;
struct Parameters;

// Snapshot of what the critic needs every tick.
struct EsdfEntry
{
  const EsdfGrid  * grid{nullptr};
  const Parameters * params{nullptr};
  // Robot pose in the odom/navigation frame at ESDF capture time.
  // Used by EsdfCritic to transform odom-frame trajectory poses to base_link.
  double robot_x{0.0};
  double robot_y{0.0};
  double robot_yaw{0.0};
};

// Registry key = ConstrainedController plugin_name_ (e.g. "ConstraintFollowPath").
// EsdfCritic derives it from parent_name_ by stripping the ".mppi" suffix.
class EsdfRegistry
{
public:
  static EsdfRegistry & instance()
  {
    static EsdfRegistry reg;
    return reg;
  }

  void set(const std::string & key, EsdfEntry e)
  {
    std::lock_guard<std::mutex> lock(mx_);
    entries_[key] = e;
  }

  EsdfEntry get(const std::string & key)
  {
    std::lock_guard<std::mutex> lock(mx_);
    auto it = entries_.find(key);
    return (it != entries_.end()) ? it->second : EsdfEntry{};
  }

  void remove(const std::string & key)
  {
    std::lock_guard<std::mutex> lock(mx_);
    entries_.erase(key);
  }

private:
  EsdfRegistry() = default;
  std::mutex mx_;
  std::unordered_map<std::string, EsdfEntry> entries_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__ESDF_REGISTRY_HPP_
