package org.firstinspires.ftc.teamcode.PATCHY.PIPELINES;
//shart
import android.graphics.Canvas;
import android.util.Log;

import org.firstinspires.ftc.onbotjava.handlers.file.NewFile;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class bluepropPipeline implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double blueThreshold = 0.06;
    double rightboxBlueThreshold = 0.1;

    String outStr = "left";

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0,0),
            new Point(426,720)
    );
    static final Rect MIDDLE_RECTANGLE = new Rect(
            new Point(427,0),
            new Point(854,720)
    );
    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(855, 0),
            new Point(1280, 720)
    );
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        //87 and 98 do NOT work
        //90 and 105 DID not work for right, worked for others
        Scalar blueLower = new Scalar(85, 100, 50); //Wraps around Color Wheel
        Scalar blueHigher = new Scalar(115, 255, 255);

        Core.inRange(testMat, blueLower, blueHigher, finalMat);

        testMat.release();
        lowMat.release();
        highMat.release();

        //Log.e("Testtesttest", String.format("width=%d, height=%d", finalMat.width(), finalMat.height()));

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];
        double middleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;
        double averagedMiddleBox = middleBox / MIDDLE_RECTANGLE.area() / 255;

        /*if (averagedLeftBox > blueThreshold && averagedRightBox > blueThreshold){
           outStr = "center";
        } else if (averagedLeftBox > averagedRightBox){
            outStr = "left";
        } else if(averagedRightBox > averagedLeftBox){
            outStr = "right";
        } else {
            outStr = "center";
        }*/

        if (averagedLeftBox > averagedRightBox && averagedLeftBox > averagedMiddleBox) {
            outStr = "left";
        } else if (averagedRightBox > averagedMiddleBox && averagedRightBox > averagedLeftBox) {
            outStr = "right";
        } else {
            outStr = "center";
        }

        // comment this stuff out when done
        finalMat.copyTo(frame);
        Imgproc.rectangle(frame, LEFT_RECTANGLE, new Scalar(255, 255, 255), 7);
        Imgproc.rectangle(frame, RIGHT_RECTANGLE, new Scalar(255, 255, 255), 7);
        Imgproc.rectangle(frame, MIDDLE_RECTANGLE, new Scalar(255, 255, 255), 7);
        //end comment

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String getPropPosition(){
        return outStr;
    }
}
