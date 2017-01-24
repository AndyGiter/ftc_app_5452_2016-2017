package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 5452 Main on 1/24/2017.
 */
@TeleOp(name="Josh TeleOp", group="Testing")
public class peteboyscoolteleop extends LinearOpMode {

    DcMotor left1;
    DcMotor left2;

    DcMotor right1;
    DcMotor right2;

    DcMotor shooter;

    DcMotor collector;

    public void runOpMode() {


        initialize();
        waitForStart();
        while (opModeIsActive()) {


            if (gamepad1.a) {
                left1.setPower(0.2);
                left2.setPower(0.2);
            } else {
                left1.setPower(0);
                left2.setPower(0);
            }

            if (gamepad1.x) {
                right1.setPower(0.2);
                right2.setPower(0.2);
            } else {
                right1.setPower(0);
                right2.setPower(0);
            }
            if (gamepad1.left_bumper){
                shooter.setPower(-0.1);
            }
            if (gamepad1.right_bumper)
                collector.setPower(0.7);
            else {
                collector.setPower(0);
            }

            if (gamepad1.y) {
                left1.setPower(0.7);
                left2.setPower(0.7);

                right1.setPower(-0.7);
                right2.setPower(-0.7);
            } else {
                left1.setPower(0);
                left2.setPower(0);
                right1.setPower(0);
                right2.setPower(0);
            }
        }
    }

    public void initialize() {
        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        shooter = hardwareMap.dcMotor.get("snail");

        collector = hardwareMap.dcMotor.get("feed");

        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
//bippity boppity boo