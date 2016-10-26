//This code will at first take heavy inspiration from some programs in bchay/3785-RESQ-Code but mostly CameraPreview.java. Thanks to bchay.
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.hardware.Camera;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.io.IOException;

@SuppressWarnings("deprecation")
public class CameraPreview implements SurfaceHolder.Callback {
    static Camera camera;
    static SurfaceView surfaceView;
    static SurfaceHolder surfaceHolder;
    static boolean viewing = false;

    static int redAvg = 100;
    static int blueAvg = 100;
    static int greenAvg = 100;

    static int redPixelCount = 0;
    static int bluePixelCount = 0;

    static float firstHUE = 128;


    CameraPreview(SurfaceView surfaceView) {
        this.surfaceView = surfaceView;
        this.surfaceHolder = surfaceView.getHolder();
        surfaceHolder.addCallback(this);
    }

    public static void start() {
        if(!viewing) {
            camera = Camera.open();
            camera.setDisplayOrientation(90);
            Camera.Parameters parameters = camera.getParameters();
            int maxZoom = parameters.getMaxZoom();
            parameters.setZoom(maxZoom);
            camera.setParameters(parameters);
            if (camera != null) {
                try {
                    camera.setPreviewDisplay(surfaceHolder);
                    camera.startPreview();
                    viewing = true;
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public static void stop() {
        if(camera != null && viewing){
            camera.stopPreview();
            camera.release();
            camera = null;
            viewing = false;
        }
    }

    public static void capture() {
        if(camera != null && viewing) {
            camera.takePicture(null, null, callback);
        }
    }

    public static Camera.PictureCallback callback = new Camera.PictureCallback() {
        @Override
        public void onPictureTaken(byte[] data, Camera camera) {
            redAvg = 0;
            blueAvg = 0;
            greenAvg = 0;

            redPixelCount = 0;
            bluePixelCount = 0;

            final int startY = 0;
            final int endY   = 80;
            final int startX = 0;
            final int endX   = 80;


            Bitmap bitmapPicture = BitmapFactory.decodeByteArray(data, 0, data.length);
            Bitmap bitmap = Bitmap.createBitmap(bitmapPicture);
            for(int y = startY; y <= endY; y++) {
                for (int x = startX; x <= endX; x++) {
                    int pixel = bitmap.getPixel(x, y);
                    int red = Color.red(pixel);
                    int blue = Color.blue(pixel);
                    int green = Color.green(pixel);

                    float[] hsv = new float[3];
                    Color.RGBToHSV(red, green, blue, hsv); //HSV. hsv[0] is Hue [0 .. 360) hsv[1] is Saturation [0...1] hsv[2] is Value [0...1]

                    if((hsv[0] > 0 && hsv[0] < 60) || hsv[0] > 300) {
                        //pixel is red
                        redPixelCount++;
                    } else if (hsv[0] > 240 && hsv[0] < 300) {
                        //pixel is blue
                        bluePixelCount++;
                    }

                    redAvg += Color.red(pixel);
                    blueAvg += Color.blue(pixel);
                    greenAvg += Color.green(pixel);

                    if(x==80 && y==80)
                    {
                        firstHUE = hsv[0];
                    }
                }
            } // end of for loop
            redAvg /= 80 * 80;
            blueAvg /= 80 * 80;
            greenAvg /= 80 * 80;
        }
    };

    public static int colorDetected() {
        return redAvg > blueAvg ? Color.RED : Color.BLUE;
    }

    public static int colorDetectedHSV() {
        return redPixelCount > bluePixelCount ? Color.RED : Color.BLUE;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }
}