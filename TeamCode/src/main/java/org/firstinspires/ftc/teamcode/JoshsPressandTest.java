package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
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
@Autonomous (name = "Josh's Dank Press and Test", group = "Auto")

public class JoshsPressandTest extends LinearOpMode {

    DcMotor left1;

    DcMotor right1;

    DcMotor left2;

    DcMotor right2;

    ColorSensor colorSensor1;

    public void runOpMode() throws InterruptedException{

        waitForStart();

        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        colorSensor1 = hardwareMap.colorSensor.get("front");

        colorSensor1.setI2cAddress(I2cAddr);

        left1.setTargetPosition(30000);
        right1.setTargetPosition(30000);
        left2.setTargetPosition(30000);
        right2.setTargetPosition(30000);

        if (colorSensor1.blue();
        left1.setTargetPosition(3000);
        right1.setTargetPosition(3000);
        left2.setTargetPosition(3000);
        right2.setTargetPosition(3000);

        if (colorSensor1.red();
        left1.setTargetPosition(0);
        right1.setTargetPosition(0);
        left2.setTargetPosition(0);
        right2.setTargetPosition(0);





    }

}
