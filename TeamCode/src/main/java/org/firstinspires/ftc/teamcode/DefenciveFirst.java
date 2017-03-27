package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 3/27/2017.
 */

@Autonomous(name = "First Beacon Defence", group ="auto")
public class DefenciveFirst extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initAndWait(DcMotor.RunMode.RUN_USING_ENCODER, true);
        leftArmServo.setPosition(1);

        //moveShootMove(-0.5, 1440 * 2.95, 1440 * 1.8);

        moveShootMoveTime(-0.5, 5, 2);

        /*
        turn(MAX_TURN_SPEED, 90); // Turn the ball away

        servoPosInit();

        turn(MAX_TURN_SPEED, -45);

        */
    }

    public void moveShootMoveTime(double speed, double totalTime, double timeBeforeShoot)
    {
        right1.setPower(speed);
        right2.setPower(speed);
        right3.setPower(speed);
        left1.setPower(speed);
        left2.setPower(speed);
        left3.setPower(speed);

        double time = getRuntime();

        while(getRuntime()-time < timeBeforeShoot && opModeIsActive()){}

        shootThreaded();

        while(getRuntime()-time < totalTime && opModeIsActive()){}

        right1.setPower(0);
        right2.setPower(0);
        right3.setPower(0);
        left1.setPower(0);
        left2.setPower(0);
        left3.setPower(0);
    }
}

