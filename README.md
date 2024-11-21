# navigation2
navigation2 is a new repository that was forked from the ROS Navigation organization to provide a local instance for development if minor changes were required to the “nav2_route_server” branch, an experimental branch used to define structured paths (i.e. road networks) for mobile robots, for it to be used with the C1T system. Updates to navigation2 focus on ensuring compatibility with ROS 2 Humble and addressing key issues related to map files and configurations.

## Changes made from upstream

* Reverted to the deprecated `create_client()` service factory-function overload that uses `rmw_qos_profile_services_default`. The replacement overload that uses `rclcpp::SystemDefaultsQoS()` was never back-ported to Humble. See [ros-planning/navigation2#3287](https://github.com/ros-planning/navigation2/issues/3287) for more information.
* Removed `launch.substitution` features that are unavailable in Humble.
* Replaced `#include "cp_bridge/cv_bridge.hpp"` with `#include "cp_bridge/cv_bridge.h"` since the former header file is unavailable in Humble.
* Removed GitHub workflows and CI configurations because we don't need them.
* Removed issue and PR templates in place of our own.

## Contribution
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Contributing.md) 

## Code of Conduct 
Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/ATTRIBUTION.txt) 

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/docs/License.md) 

## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)
