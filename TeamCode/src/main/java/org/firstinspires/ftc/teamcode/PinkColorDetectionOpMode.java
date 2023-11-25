package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "PinkColorDetectionOpMode", group = "TeleOp")
public class PinkColorDetectionOpMode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera webcam;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        
        // Create an OpenCvCamera instance
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        
        // Set the pipeline to an instance of your color detection pipeline
        webcam.setPipeline(new PinkColorDetectionPipeline());
        
        // Open the connection to the camera device
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.update();

        // Your teleop logic goes here
    }

    @Override
    public void stop() {
        // Close the connection to the camera device when the op mode is stopped
        webcam.closeCameraDevice();
    }

    // Define your color detection pipeline class
    public static class PinkColorDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to HSV color space
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            // Define the rang//share of pink color in HSV
            Scalar lowerBound = new Scalar(150, 50, 50);
            Scalar upperBound = new Scalar(180, 255, 255);
            
            // Threshold the image to keep only the pixels within the pink color range

            Core.inRange(input, lowerBound, upperBound, input);

            return input;
            
        }
    }
}
