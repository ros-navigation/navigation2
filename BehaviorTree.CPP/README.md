![License MIT](https://img.shields.io/dub/l/vibe-d.svg)
![Version](https://img.shields.io/badge/version-v2.0-green.svg)
[![Build Status](https://travis-ci.org/BehaviorTree/BehaviorTree.CPP.svg?branch=master)](https://travis-ci.org/BehaviorTree/BehaviorTree.CPP)

# About BehaviorTree.CPP

This  __C++__ library provides a framework to create BehaviorTrees.
It was designed to be flexible, easy to use and fast.

Even if our main use-case is __robotics__, you can use this library to build
__AI for games__, or to replace Finite State Machines in you application.

__BehaviorTree.CPP__ has many interesting features, when compared to other implementations:

- It makes asynchronous Actions, i.e. non-blocking, a first-class citizen.
- It allows the creation of trees at run-time, using a textual representation (XML).
- You can link staticaly you custom TreeNodes or convert them into plugins 
which are loaded at run-time.
- It includes a __logging/profiling__ infrastructure that allows the user 
to visualize, record, replay and analyze state transitions.

Documentation
------------

https://behaviortree.github.io/BehaviorTree.CPP/

GUI Editor
------------

Editing a BehaviorTree is as simple as editing a XML file in your favourite text editor.

If you are looking for a more fancy graphical user interface, check 
[Groot](https://github.com/BehaviorTree/Groot) out.

![Groot screenshot](groot-screenshot.png)


Acknowledgement
------------
This library was developed at  **Eurecat** (main author, Davide Faconti) in a joint effort
with the **Italian Institute of Technology** (Michele Colledanchise).

It is one of the main components of [MOOD2Be](https://eurecat.org/es/portfolio-items/mood2be/),
which is one of the six **Integrated Technical Projects (ITPs)** selected from the
[RobMoSys first open call](https://robmosys.eu/itp/) and it received funding from the European
Union’s Horizon 2020 Research and Innovation Programme.

Further readings
---------------

- Introductory article: [Behavior trees for AI: How they work](http://www.gamasutra.com/blogs/ChrisSimpson/20140717/221339/Behavior_trees_for_AI_How_they_work.php)

- **How Behavior Trees Modularize Hybrid Control Systems and Generalize Sequential Behavior Compositions, the Subsumption Architecture,
and Decision Trees.** Michele Colledanchise and Petter Ogren. IEEE Transaction on Robotics 2017.

- **Behavior Trees in Robotics and AI**, published by CRC Press Taylor & Francis, available for purchase
(ebook and hardcover) on the CRC Press Store or Amazon.

 The Preprint version (free) is available here: https://arxiv.org/abs/1709.00084


License
-------
The MIT License (MIT)

Copyright (c) 2014-2018 Michele Colledanchise
Copyright (c) 2018 Davide Faconti

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
