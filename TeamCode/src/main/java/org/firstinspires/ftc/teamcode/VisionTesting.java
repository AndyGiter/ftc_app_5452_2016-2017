package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CameraPreview;

import android.app.Activity;
import android.content.Context;
import android.graphics.Camera;
import android.view.SurfaceView;

/**
 * Created by mlowery2 on 10/13/2016.
 *
 * This code will at first take heavy inspiration from some programs in bchay/3785-RESQ-Code but mostly CameraPreview.java. Thanks to bchay.
 */

@TeleOp(name="Vision Testing", group="Testing")
public class VisionTesting extends LinearOpMode {

    public void runOpMode() throws InterruptedException
    {
        CameraPreview c = new CameraPreview((SurfaceView) ((Activity) hardwareMap.appContext).findViewById(R.id.surfaceView));
        c.start();

        waitForStart();

        c.capture();
        telemetry.addData("Color Detected", "RGB: " + c.colorDetected());
        telemetry.addData("Color Detected", "HSV: " + c.colorDetectedHSV());
        telemetry.update();
        while(opModeIsActive()){Thread.sleep(50);}
    }
}
