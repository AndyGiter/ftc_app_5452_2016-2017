package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by mlowery2 on 1/24/2017.
 *
 * This is just a program for me to learn how to use the touch sensor because there arent any sample programs
 */
@Disabled
@Autonomous(name = "Touch Testing", group ="testing")
public class TouchTester extends LinearBase {

    final int BLUE_LED_CHANNEL = 0;
    final int RED_LED_CHANNEL = 1;

    DeviceInterfaceModule dim;                  // Device Object
    DigitalChannel        digIn;                // Device Object

    public void runOpMode() throws InterruptedException
    {
        // get a reference to a Modern Robotics DIM, and IO channels.
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping
        digIn  = hardwareMap.get(DigitalChannel.class, "touch");     //  Use generic form of device mapping

        digIn.setMode(DigitalChannelController.Mode.INPUT);          // Set the direction of each channel

        initalize(true);
        waitForStart();

        while(opModeIsActive())
        {

            // Display input pin state on LEDs
            if ((digIn.getState() == touch.getState())) {
                dim.setLED(RED_LED_CHANNEL, true);
                dim.setLED(BLUE_LED_CHANNEL, false);
            }
            else {
                dim.setLED(RED_LED_CHANNEL, false);
                dim.setLED(BLUE_LED_CHANNEL, true);
            }

            if(gamepad1.b)
            {

                shooter.setPower(0.9);
            }

            else
            {
                shooter.setPower(0);
            }

            telemetry.addData("digin", digIn.getState());
            telemetry.addData("touch", touch.getState());
            telemetry.addData("LED",   digIn.getState() == touch.getState() ? "Red" : "Blue" );
            telemetry.update();

        }
    }
}
