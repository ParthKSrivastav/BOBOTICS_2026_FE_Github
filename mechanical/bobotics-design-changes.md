## Design Changes and Their Impact

Our robot has developed through several versions, each helping us understand what worked, what needed improving, and when a different approach was necessary.

### Marks 1–5: Weight and Motor Limitations

Our first five designs could not move reliably. The robots were too heavy for the motors we were using, which highlighted the need to balance the robot’s weight with the available motor power.

### Marks 6–8: First Movement and Steering Challenges

Mark 6 was our first major **breakthrough**: we got the robot moving. Across Marks 6–8, however, turning remained inconsistent. Although the robot could drive, its steering was not accurate enough to navigate the course reliably.

### Marks 9–11: Moving to Raspberry Pi

For Mark 9, we moved away from LEGO SPIKE and adopted a Raspberry Pi-based system using an RC vehicle we nicknamed the “yellow truck.” The truck initially weighed approximately 400 g.

Marks 9–11 used the same platform, which delivered much more accurate movement and strong performance in the first round. At approximately 450 g in its developed configuration, the robot could complete the Open Challenge in around 40 seconds, with 10 successful runs out of 10 in our testing.

However, the code was relatively basic. Its limitations became more apparent when we began adding obstacle-handling logic, showing that we needed a more capable software approach.

### Mark 12: A New Vehicle and a Software Rebuild

For Mark 12, we replaced the yellow truck with a smaller kei truck and introduced a new control approach. This meant revisiting much of our earlier work and rebuilding the software around the new platform.

Although this was a challenging step backwards initially, we remained determined. Over approximately one month, we rebuilt the code and integrated ROS, eventually getting the robot to complete three laps.

### Current Version: Open and Obstacle Challenges

Our current robot includes logic for both the Open and Obstacle Challenges. The Open Challenge remains reliable, with 10 successful runs out of 10 in our testing.

Obstacle performance is still less consistent in our home setup. The practice mat is heavily creased, making it harder for the robot to travel smoothly and turn accurately. Further testing on a flatter surface will help us distinguish problems caused by the course condition from those that still need improvements in the robot’s control logic.
