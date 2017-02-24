package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by 5452 Main on 1/31/2017.
 */
@Disabled
@Autonomous (name = "Josh's Dank Press and Test", group = "Auto")
public class JoshsPressandTest extends LinearOpMode {

    DcMotor left1;

    DcMotor right1;

    DcMotor left2;

    DcMotor right2;

    ColorSensor Sensei;

    private I2cAddr i2cAddrSensei = I2cAddr.create8bit(0x4c);

    public void runOpMode() throws InterruptedException{

        waitForStart();

        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        Sensei = hardwareMap.colorSensor.get("sensei");

        Sensei.setI2cAddress(i2cAddrSensei);

        left1.setPower(0.7);
        right1.setPower(0.7);
        left2.setPower(0.7);
        right2.setPower(0.7);
        sleep(3000);

        if (Sensei.blue() > Sensei.red())
        {
            left1.setPower(0.7);
            right1.setPower(0.7);
            left2.setPower(0.7);
            right2.setPower(0.7);
            sleep(2000);
        }

        else if(Sensei.blue() < Sensei.red())
        {
            left1.setPower(0);
            right1.setPower(0);
            left2.setPower(0);
            right2.setPower(0);
        }





    }

}
