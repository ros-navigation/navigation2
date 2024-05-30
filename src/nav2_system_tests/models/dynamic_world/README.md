# Dynamic World Model

A world model with a ground plane, 20x20 meter empty room, and 9 dynamic obstacles.

## How to Add a Dynamic Obstacle

dynamic_world/world.model  

    <model name="DynamicObstacle">
          -- Position of the new obstacle
           <pose>-4 4 0.15 0 0 0</pose>
           <include>
            <uri>model://dynamic_obstacle</uri>
          </include>
    </model>

