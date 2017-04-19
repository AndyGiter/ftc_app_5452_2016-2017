## Teamcode Folder
Thanks for looking at the code for team 5452: Robot to the Knee. This code was written for [2016-2017 Velocity Vortex](https://www.youtube.com/watch?v=iQLrcQbm8cg "Game video"). This folder has all the code that needed to run on the robot at the end of the season. Programs that were not needed were deleted. All programs currently in this folder were written by Max Lowery and some that went unused (and deleted) were written by Josh Keller.

## Program Listing
* __LinearBase:__ The program that contained all the initialisation and movement functions.

* __BasicTeleop:__ Our teleop for the year.

* __BeaconBallBlue & BleaconBallRed:__ Our primary scoring autonomous that shot two balls, pressed a beacon to our color, knocked off the cap ball, and partially parked on the center vortex.

* __MoveShoot & MoveShootOneBall:__ Two of our support autonomous that started in the corner, moved forward after some time, and shot either one or two balls into the center vortex.

* __MoveShootBump & MoveShootBumpOneBall:__ The other two support autonomous that do the same as the previous two, and also knock off the cap ball and partially park on the center vortex.

* __SensorTester:__ This program has done a lot of differnt things though the year and was made as a sensor playground of sorts. Currently just displays the readings front range senor in telemetry.

* __ColorTesting:__ Displays color sensor readings in telemetry. We had a lot of color sensor problems at the start of the year with placement on the robot and I2C and this program came in handy.