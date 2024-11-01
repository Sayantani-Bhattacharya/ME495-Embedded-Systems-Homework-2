1. The code works for both the cases, and can be tested with (values what I last checked with, should work for other cases also):

    > ros2 launch turtle_brick turtle_arena.launch.xml
    
    Case1:
    > ros2 service call /place turtle_brick_interfaces/srv/Place "{pose: {x: 3.0, y: 2.0, z: 2.0}}"
    
    Case2:
    > ros2 service call /place turtle_brick_interfaces/srv/Place "{pose: {x: 3.0, y: 2.0, z: 10.0}}"
    > 
    > ros2 service call /drop turtle_brick_interfaces/srv/Gravity "{gravity: {data: 9.81}}"

2. Also, all the three launch files work, the arg and rviz defaults needs to be changed for running them individually though.
3. The going to home case, has a direction issue, so I have commented it.
4. The tilt condition is not completely written.

-Thankyou
