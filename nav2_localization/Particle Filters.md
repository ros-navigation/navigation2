# A Comparison of Particle Filter Libraries
This document summarizes the pros and cons  of readily available particle filter libraries. The aim is to identify if any of these libraries is suitable for use in the Localization Framework.

| Library                                                             |    Pros     |   Cons            |   Dependencies |
| -----------                                                         | ----------- | ---------         |   ------------ |
| [MIT Race Car](https://github.com/mit-racecar/particle_filter)      |             | <ul><li>Written in Python</li></ul> |                |
| [Robotology](https://github.com/robotology/bayes-filters-lib)       | <ul><li>Good documentation</li><li>Implements KFs</li></ul>  | <ul><li>The structure is confusing</li><li>No adaptive resampling</li></ul>  |   <ul><li>Eigen</li></ul>   |
| [libPF](https://github.com/stwirth/libPF) |   <ul><li>Very simple</li><ul> | <ul><li>The movement and measurement model interfaces are not suitable for what we're trying to do.</li><ul> |   |
| [Rookfighter](https://github.com/Rookfighter/bayes-filter-cpp)    |  <ul><li>Very simple</li><li>The motion and measurement model are very well suited for what we already have</li><li>Implements KFs</li><ul>   |   <ul><li>No adaptive resampling</li></ul>     | <ul><li>Eigen</li></ul> |
| [MRPT](https://github.com/MRPT/mrpt/) |   <ul><li>Well maintained and documented</li><ul> |   <ul><li>Way too complicated for what we're trying to do.</li><li>The movement and measurement model interfaces are not suitable for what we're trying to do.</li><ul>   | <ul><li>Eigen</li></ul> |
| [amcl3d](https://github.com/fada-catec/amcl3d) | <ul><li>Fairly simple</li><ul>   |   <ul><li>The motion and measurement model are effectively part of the pf code</li><ul>   |   <ul><li>PCL</li></ul> |
| [tbrown pf](https://github.com/tbrown122387/pf) |     |   <ul><li>The structure is quite confusing</li><ul>   |   <ul><li>Eigen</li><li>Boost</li><li>Catch2 (unit testing)</li><ul> 