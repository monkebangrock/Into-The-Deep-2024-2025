package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.core.Mean;


public class BlueCam implements VisionProcessor{
    public Rect r1c1 = new Rect(0, 0, 320, 180);
    public Rect r1c2 = new Rect(320, 0, 320, 180);
    public Rect r1c3 = new Rect(640, 0, 320, 180);
    public Rect r1c4 = new Rect(960, 0, 320, 180);
    public Rect r2c1 = new Rect(0, 180, 320, 180);
    public Rect r2c2 = new Rect(320, 180, 320, 180);
    public Rect r2c3 = new Rect(640, 180, 320, 180);
    public Rect r2c4 = new Rect(960, 180, 320, 180);
    public Rect r3c1 = new Rect(0, 360, 320, 180);
    public Rect r3c2 = new Rect(320, 360, 320, 180);
    public Rect r3c3 = new Rect(640, 360, 320, 180);
    public Rect r3c4 = new Rect(960, 360, 320, 180);
    public Rect r4c1 = new Rect(0, 540, 320, 180);
    public Rect r4c2 = new Rect(320, 540, 320, 180);
    public Rect r4c3 = new Rect(640, 540, 320, 180);
    public Rect r4c4 = new Rect(960, 540, 320, 180);
    public int bluestCol;
    public int bluestRow;
    Rect[][] boxes = {{r1c1, r1c2, r1c3, r1c4}, {r2c1, r2c2, r2c3, r2c4},
                        {r3c1, r3c2, r3c3, r3c4}, {r4c1, r4c2, r4c3, r4c4}};
    double bluest;
    Rect bluestLocation;
    int cnt=0;

    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration){
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        bluest = getAvgBlueness(frame, boxes[0][0]);
        bluestLocation = boxes[0][0];
        for (int i = 0; i < 4; i++) {
            for (int p = 0; p < 4; p++) {
                if (getAvgBlueness(frame, boxes[i][p]) > bluest) {
                    bluest = getAvgBlueness(frame, boxes[i][p]);
                    bluestLocation = boxes[i][p];
                    bluestRow = i;
                    bluestCol = p;
                }
            }
        }

        return bluestLocation;
    }

    protected double getAvgBlueness(Mat input, Rect rect){
        Mat submat = input.submat(rect);
        Scalar lowerBlue = new Scalar(0, 0, 75, 0);
        Scalar upperBlue = new Scalar(50, 50, 255, 255);

        Mat mask = new Mat();
        Core.inRange(submat, lowerBlue, upperBlue, mask);

                /*
        // Calculate the percentage of red pixels
        int totalPixels = submat.rows() * submat.cols();
        int redPixels = Core.countNonZero(mask);
        double rednessPercentage = (redPixels / (double) totalPixels) * 100;
        return rednessPercentage;
        */

        double meanBlue = Core.mean(submat, mask).val[0];
        return meanBlue;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onScreenWidth, int onScreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonselectedPaint = new Paint(selectedPaint);
        nonselectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawR1C1 = makeGraphicsRect(r1c1, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR1C2 = makeGraphicsRect(r1c2, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR1C3 = makeGraphicsRect(r1c3, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR1C4 = makeGraphicsRect(r1c4, scaleBmpPxToCanvasPx);
        /*android.graphics.Rect drawR2C1 = makeGraphicsRect(r2c1, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR2C2 = makeGraphicsRect(r2c2, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR2C3 = makeGraphicsRect(r2c3, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR2C4 = makeGraphicsRect(r2c4, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR3C1 = makeGraphicsRect(r3c1, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR3C2 = makeGraphicsRect(r3c2, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR3C3 = makeGraphicsRect(r3c3, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR3C4 = makeGraphicsRect(r3c4, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR4C1 = makeGraphicsRect(r4c1, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR4C2 = makeGraphicsRect(r4c2, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR4C3 = makeGraphicsRect(r4c3, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawR4C4 = makeGraphicsRect(r4c4, scaleBmpPxToCanvasPx);*/


        //ill optimize this later but for testing purposes this is it for now, only 1st row will indicate currently
        canvas.drawRect(drawR1C1, nonselectedPaint);
        canvas.drawRect(drawR1C2, nonselectedPaint);
        canvas.drawRect(drawR1C3, nonselectedPaint);
        canvas.drawRect(drawR1C4, nonselectedPaint);

        if(bluestLocation == boxes[0][0]) {
            canvas.drawRect(drawR1C1, selectedPaint);
        }
        else if(bluestLocation == boxes[0][1]) {
            canvas.drawRect(drawR1C2, selectedPaint);
        }
        else if(bluestLocation == boxes[0][2]) {
            canvas.drawRect(drawR1C3, selectedPaint);
        }
        else if(bluestLocation == boxes[0][3]){
            canvas.drawRect(drawR1C4, selectedPaint);
        }
    }

    public String getBox(){
        String box;
        box = bluestRow + ", " + bluestCol;
        return box;
    }

    public Selected getSelection(){
        return selection;
    }

    public enum Selected{
        R1C1,
        R1C2,
        R1C3,
        R1C4,
        NONE
    }
}
