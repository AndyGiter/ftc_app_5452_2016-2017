package org.firstinspires.ftc.teamcode;


        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 5452 Main on 2/6/2017.
 */
@TeleOp(name="Food TeleOp", group="Testing")
public class FoodBotTeleop extends LinearOpMode {

    DcMotor left1;
    DcMotor left2;

    DcMotor right1;
    DcMotor right2;

    public void runOpMode() {

        initialize();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.y) {
                right1.setPower(.7);
                right2.setPower(.7);
                left1.setPower(.7);
                left2.setPower(.7);
            } else if (gamepad1.x) {
                left2.setPower(-.7);
                left1.setPower(-.7);
                right1.setPower(.7);
                right2.setPower(.7);
            } else if (gamepad1.a) {
                right2.setPower(-.7);
                right1.setPower(-.7);
                left1.setPower(-.7);
                left2.setPower(-.7);
            } else if (gamepad1.b) {
                left1.setPower(.7);
                left2.setPower(.7);
                right1.setPower(-.7);
                right2.setPower(-.7);
            } else {
                left2.setPower(0);
                right1.setPower(0);
                left1.setPower(0);
                right2.setPower(0);
            }
        }
    }

    public void initialize() {
        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");


        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}

    