<!-- Please fill out the following pull request template for non-trivial changes to help us process your PR faster and more efficiently.-->

---

## Basic Info

| Info | Please fill out this column |
| ------ | ----------- |
| Ticket(s) this addresses   | (add tickets here #1) |
| Primary OS tested on | (Ubuntu, MacOS, Windows) |
| Robotic platform tested on | (Steve's Robot, gazebo simulation of Tally, hardware turtlebot) |
| Does this PR contain AI generated software? | (No; Yes and it is marked inline in the code) |

---

## Description of contribution in a few bullet points

<!--
* I added this neat new feature
* Also fixed a typo in a parameter name in nav2_costmap_2d
-->

## Description of documentation updates required from your changes

<!--
* Added new parameter, so need to add that to default configs and documentation page
* I added some capabilities, need to document them
-->

---

## Future work that may be required in bullet points

<!--
* I think there might be some optimizations to be made from STL vector
* I see alot of redundancy in this package, we might want to add a function `bool XYZ()` to reduce clutter
* I tested on a differential drive robot, but there might be issues turning near corners on an omnidirectional platform
-->

#### For Maintainers: <!-- DO NOT EDIT OR REMOVE -->
- [ ] Check that any new parameters added are updated in docs.nav2.org
- [ ] Check that any significant change is added to the migration guide
- [ ] Check that any new features **OR** changes to existing behaviors are reflected in the tuning guide
- [ ] Check that any new functions have Doxygen added
- [ ] Check that any new features have test coverage
- [ ] Check that any new plugins is added to the plugins page
- [ ] If BT Node, Additionally: add to BT's XML index of nodes for groot, BT package's readme table, and BT library lists
