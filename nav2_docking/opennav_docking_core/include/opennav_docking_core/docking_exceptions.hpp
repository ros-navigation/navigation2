// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING_CORE__DOCKING_EXCEPTIONS_HPP_
#define OPENNAV_DOCKING_CORE__DOCKING_EXCEPTIONS_HPP_

#include <string>
#include <memory>
#include <stdexcept>

namespace opennav_docking_core
{

/**
 * @class DockingException
 * @brief Abstract docking exception
 */
class DockingException : public std::runtime_error
{
public:
  explicit DockingException(const std::string & description)
  : std::runtime_error(description) {}
};

/**
 * @class DockNotInDB
 * @brief Dock was not found in the provided dock database
 */
class DockNotInDB : public DockingException
{
public:
  explicit DockNotInDB(const std::string & description)
  : DockingException(description) {}
};

/**
 * @class DockNotValid
 * @brief Dock plugin provided in the database or action was invalid
 */
class DockNotValid : public DockingException
{
public:
  explicit DockNotValid(const std::string & description)
  : DockingException(description) {}
};

/**
 * @class FailedToStage
 * @brief Failed to navigate to the staging pose
 */
class FailedToStage : public DockingException
{
public:
  explicit FailedToStage(const std::string & description)
  : DockingException(description) {}
};

/**
 * @class FailedToDetectDock
 * @brief Failed to detect the charging dock
 */
class FailedToDetectDock : public DockingException
{
public:
  explicit FailedToDetectDock(const std::string & description)
  : DockingException(description) {}
};

/**
 * @class FailedToControl
 * @brief Failed to control into or out of the dock
 */
class FailedToControl : public DockingException
{
public:
  explicit FailedToControl(const std::string & description)
  : DockingException(description) {}
};

/**
 * @class FailedToCharge
 * @brief Failed to start charging
 */
class FailedToCharge : public DockingException
{
public:
  explicit FailedToCharge(const std::string & description)
  : DockingException(description) {}
};

}  // namespace opennav_docking_core

#endif  // OPENNAV_DOCKING_CORE__DOCKING_EXCEPTIONS_HPP_
