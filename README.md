# ME495 Embedded Systems Homework 2
Author: Sayantani Bhattacharya

${The pkg is used to visualise the turtle bot inside an arena, and has services to place and drop a brick. And control node is used to catch the dropping brick}

## Quickstart
1. Use `ros2 launch ${turtle_brick} ${run_turtle.launch.xml} ${options}` to start the arena and turtle simulation.

2. Use ${ros2 service call /place turtle_brick_interfaces/srv/Place "{pose: {x: 5.0, y: 2.0, z: 2.0}}"} to place the brick.

3. Use ${ros2 service call /drop turtle_brick_interfaces/srv/Gravity "{gravity: {data: 40.0}}"} to drop the brick.

4. Here is a video of the turtle when the brick is within catching range
   ${embed video here, it must be playable on github. Upload the video as an issue and link to it}

5. Here is a video of the turtle when the brick cannot be caught

   ${embed video here, it must be playable on github. Upload the video as an issue and link to it}