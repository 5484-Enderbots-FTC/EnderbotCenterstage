package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

class pipeline extends OpenCvPipeline {
    public Mat processFrame(Mat input) {

        Mat YCbCr = new Mat();
        Mat leftcrop;
        Mat rightcrop;
        double leftavgfin;
        double rightavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(1, 1, 640, 360);
        Rect rightRect = new Rect(1, 1, 640, 360);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftcrop = YCbCr.submat(leftRect);
        rightcrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftcrop,leftcrop,2);
        Core.extractChannel(rightcrop,rightcrop, 2);

        Scalar leftavg = Core.mean(leftcrop);
        Scalar rightavg = Core.mean(rightcrop);

        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];

        if (leftavgfin >  rightavgfin) {
            telemetry.addLine("Left");
        }
        else{
            telemetry.addLine("Right");
        }


        return (output);
    }
}





