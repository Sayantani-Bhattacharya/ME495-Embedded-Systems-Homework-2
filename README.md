# ME495 Embedded Systems Homework 2
Author: Sayantani Bhattacharya

The pkg is used to visualise the turtle bot inside an arena, and has services to place and drop a brick. And control node is used to catch the dropping brick.

## Quickstart
1. Use `$ros2 launch ${turtle_brick} ${run_turtle.launch.xml} ${options}` to start the arena and turtle simulation.

2. Use `$ros2 service call /place turtle_brick_interfaces/srv/Place "{pose: {x: 5.0, y: 2.0, z: 10.0}}"` to place the brick.

3. Use `$ros2 service call /drop turtle_brick_interfaces/srv/Gravity "{gravity: {data: 9.80}}"` to drop the brick.

4. Here is a video of the turtle when the brick is within catching range
   <a href="https://github.com/ME495-EmbeddedSystems/homework-2-Sayantani-Bhattacharya/issues/1#issue-2620761607">Watch video on GitHub</a>

5. Here is a video of the turtle when the brick cannot be caught

   ${embed video here, it must be playable on github. Upload the video as an issue and link to it}
