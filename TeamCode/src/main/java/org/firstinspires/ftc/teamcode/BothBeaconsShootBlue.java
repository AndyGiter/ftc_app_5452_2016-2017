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

    private final double MIN_DIST = 0.13;
    private final double MAX_DIST = 0.15;

    private final double TURN_SPEED = 0.2;
    private final double MOVE_SPEED = 0.65;

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        double startTime = getRuntime();

        //moveShootMove(MOVE_SPEED, 1440 * 1.2, 1440 * 0.2);

        move(-1*MOVE_SPEED, -1*1440);

        colorTelemetry(red);
        colorTelemetry(blue);
        telemetry.update();

        Thread.sleep(4250);// see if this can be shirtebed
        testBump(Color.BLUE);
    }

    public void moveShootMove(double speed, double totalDist, double distBeforeShoot) throws InterruptedException
    {
        move(speed, distBeforeShoot);

        cannon.setPosition(0);
        Thread.sleep(200);

        move(speed, totalDist - distBeforeShoot);
    }

    // the fuction assumes that the robot has already press one of the buttons on the beacon then run this function
    public void testBump(int color) throws InterruptedException
    {
        red.enableLed(false);
        blue.enableLed(false);

        colorTelemetry(red);
        colorTelemetry(blue);
        telemetry.update();

        if(color == Color.RED)
        {
            if(red.blue() > red.red() || blue.blue() > blue.red())
            {
                move(MOVE_SPEED, 1440*0.5);
                move(MOVE_SPEED, 1440*-0.55);
            }

        }
        else if(color == Color.BLUE)
        {
            if(red.red() > red.blue() || blue.red() > blue.blue())
            {
                move(MOVE_SPEED, 1440*0.5);
                move(MOVE_SPEED, 1440*-0.55);
            }
        }

        //red.enableLed(true);
        //blue.enableLed(true);
    }
    public void colorTelemetry(ColorSensor color)
    {
        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Green: ", color.green());
    }
}
