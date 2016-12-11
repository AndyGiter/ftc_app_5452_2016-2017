package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 12/1/2016.
 *
 * Id like to ask god for help to make sure this shit works
 */
@Autonomous(name="Both Beacons Shoot BLUE", group="testing")
public class BothBeaconsShootBlue extends LinearBase {

    private final double TURN_SPEED = 0.2; // try increasing this after removal of sensors
    private final double MOVE_SPEED = 0.75;

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        front.enableLed(true);
        waitForStart();
        sleep(100);

        turn(TURN_SPEED, 90);
        //pressAndTest(MOVE_SPEED, 1440, Color.BLUE);
    }


}
