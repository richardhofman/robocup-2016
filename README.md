# 2016 RoboCup Control Program
## Background
As part of the University of Canterbury Mechatronics program, I was required to work in a group of other students to develop a robot. The robot was then set against other robots
in a competition (the RoboCup), the challenge being to collect as many metal "food packages" (weights 0.25kg - 1.0kg) as possible in a five minute round. The robots were not allowed
to carry more than three weights at a time, and so were required to return to base and drop off their cargo to collect more than three weights.

## What's this, then?
This is the control software developed for our robot. In the end it turned out to be severely cut down from what was originally intended, thanks to time limitations, a
frantic redesign after some very strange electrical/data bus issues, and some poor planning on my part. Its basic mode of operation is to wander the arena, avoiding walls, and
turning towards (and trying to pick up) any weights it detects.

## ... So how'd the robot work?
Our design was fairly unique in that it only used two sensors, VL53L0X IR time-of-flight sensors (in effect, baby LIDAR), sweeping back and forth, to detect weights and obstacles at angle offsets.
We used an electromagnet attached to a crane to pick up magnets - an approach which conveniently prevented us from picking up the plastic decoy weights that were also put in the arena!

Due to some hardware issues in the hours running up to the competition, we began experiencing erratic crashes and UART/I2C bus failures, one of which rendered our robot dead in the
arena, and we didn't end up collecting any weights.

I'm uploading this code for posterity, and so I can cite it in my final design evaluation report ;-)