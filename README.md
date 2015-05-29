# PID
This is our own implementation of the PID algorithm which we will use to stabilize a ball on a horizontal beam.
The contraption used to test the code will likely use a servo motor and some sort of sensor(pot, ultrasonic sensor, or other angle sensor) and the motor will control a joint that tilts the beam this way and that until the ball reaches the desired location(middle where beam is balanced).

Unfortunately, the gyro sensor we use(MPU9150) drifts too much so we do not have enough knowledge to control it properly. Our complementary filter does not work like suspected so we decided to pause this project and pursue a lesser version(PID control Beam Balance).

