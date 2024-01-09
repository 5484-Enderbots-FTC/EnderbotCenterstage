package org.firstinspires.ftc.teamcode.PATCHY;
//shart
import android.graphics.Canvas;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class redpropPipeline implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double redThreshold = 0.6;

    String outStr = "left";

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0,0),
            new Point(640,720)
    );
    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(1280-640,0),
            new Point(1279,720)
    );
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVRedLower = new Scalar(0, 60, 125);  //Beginning of Color Wheel
        Scalar lowHSVRedUpper = new Scalar(40, 255, 255);

        Scalar redHSVRedLower = new Scalar(141, 60, 125); //Wraps around Color Wheel
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);


        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        //testMat.release();
        //lowMat.release();
        //highMat.release();

        //Log.e("Testtesttest", String.format("width=%d, height=%d", finalMat.width(), finalMat.height()));

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;

        if (averagedLeftBox > redThreshold){
            outStr = "left";
        } else if(averagedRightBox > redThreshold){
            outStr = "right";
        }else {
            outStr = "center";
        }
        // comment this stuff out when done
        finalMat.copyTo(frame);
        Imgproc.rectangle(frame, LEFT_RECTANGLE, new Scalar(255, 255, 255), 7);
        Imgproc.rectangle(frame, RIGHT_RECTANGLE, new Scalar(255, 255, 255), 7);
        //end comment

        return finalMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String getPropPosition(){
        return outStr;
    }
}
