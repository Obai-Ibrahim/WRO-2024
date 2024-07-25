## INTRODUCTION
This Repository contains all the necessary information to build a self-driving car, which can complete three laps and handle different configurations of obstacles.
The code is written with Arduino C programming language, which controls the motor's movement based on the different sensors reading and mainly the information from the pixy cam.
The Repository contains an explanation of many aspects like the strategies and the power and mobility management. In the directory (other) many files will declare the calculation and the algorithms in a better way.
These files are:
Diagrams, DC motor, Power consumption and power supply, Driver.

## Open challenge strategy:
For the open challenge, our algorithm depends basically on the distance sensors (HC-SR04) and the IMU (MPU6050) sensor, we implemented a Kalman filter to get accurate measurements of the orientation.
There are 8 ultrasonic sensors postured as shown in (found in others directory).
The front one is (0) and the back one is (4)
(2) and  (3) are the right sensors used to detect the turn in the clockwise round.
(2) and  (3) are the right sensors used to detect the turn in the clockwise round.
(1)  And (7) are installed on two sgSG-90ervo motors which move according to the Robot’s angle to guarantee good measurement of the perpendicular distance from the walls.
As the new rule states that the vehicle must not touch the outer wall, we declared a dangerous area which is 8cm from the outer wall.
The robot moves forward controlling the direction with PDthe  controller based on MPthe U6050 reading.
Obstacle challenge strategy:
We found out that planning a path for each stretch of the map and guiding the robot to follow it can be a confusing method because it needs perfect calibration for our camera and this may take a long time (maybe preparation time before the round is not sufficient).
So the algorithm in the first turn (the pillars are not known yet) will be: 
The vehicle will pass the forward section in two stages and the corner section in one stage. In the forward section, the pixy will detect the pillars location and then path the pillars one by one (the flow chart in (the others directory will declare this better))

After the first round the pillars positions are known, so we will follow the second strategy:
 By knowing the current position, current heading, target position, and target heading we can generate a polynomial to fit these parameters. Using multiple target points, we create a target path for the robot to follow, steering it towards the next point. This path accounts for the position of obstacles, mapping points to avoid them. We repeat this process until all three rounds are completed.

Note: detailed diagrams for both open and obstacle challenges are included in the directory (others)

## Mobility management
Vehicle’s chassis:
We bought a simple chassis with Ackerman for front-wheel servo steering. The kit contained four wheels, one metal base, nuts and bolts, a DC Motor, and a servo motor with an Ackerman mechanism with no sensors or additional parts. After that, we chose a suitable set of sensors and power supplies. We planned in advance to use a multi-floor design with 3D printed Ultrasonic holders and laser cut floor to have a balanced weight distribution.  
We used another DC motor because the one with the kit was not good, also we changed the rear axle and assembled the wheels with a Lego differential to make the movement better especially on turns.
To do this we printed a 3D coupler to transfer movement from the engine to the differential’s axis.
The 3D printed parts and plexi laser cut parts are included in the directory (models).
Details of the motor and the torque calculation are in the directory (others).

## Power and sense management:
The power calculations are detailed in a PDF file in the directory (scheme). The file contains an accurate calculation of the needed current and the needed supply voltage for the circuit components, in addition, some requirements were considered to choose a suitable battery to supply the sensors and actuators with sufficient power.
The used sensors are: 
1. ultrasonic sensors: dealing with walls and barriers can be enhanced with Ultrasonic Sensor. You shoot out a sound, wait to hear it echo back, and if you have your timing right, you will know how far away the wall is. Using the HC-SR04 Ultrasonic Sensor made it easy for our vehicle to avoid hitting the walls and find its best route between the colored pillars.

2. Gyroscope and accelerometer: We used MPU6050 to determine the vehicle’s rotation angle when it turns right or left. It also helps us to measure acceleration, velocity, orientation, displacement, and many other motion-related parameters of a system or object. This module also has a Digital Motion Processor inside it which is powerful enough to perform complex calculations and thus free up the work for Microcontroller. It has well documented and revised libraries available hence it’s very easy to use with famous platforms like Arduino.

3. TCRT5000Analog: helps to detect the turn if there is doubt with it as it can detect whether the color is white or nonotit is not used so much but can be useful in some cases, it is mounted with a servo motor to control its angle and to put it up when not used so it will not affect the camera readings.

4. Pixy2.1 camera: Using the second version of Pixy made the process smooth and fast with easy-to-detect errors and precise color detection. We faced many problems at the beginning as the camera could only deal with a small range of colors with almost perfect lighting conditions, but this problem was resolved by adjusting camera settings and calibrating it with more than one shade of each color. The information returned by the pixy are the detected objects with their coordinates and sizes.
