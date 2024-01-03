package org.firstinspires.ftc.teamcode.PATCHY;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

public class visiontest {

    @Autonomous(name = "Vision Test")
    public class CameraTest extends LinearOpMode {


        private VisionPortal portal;
        private org.firstinspires.ftc.teamcode.PATCHY.redpropPipeline redpropPipeline;

        @Override
        public void runOpMode() throws InterruptedException {

            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(1280, 720))
                    .setCamera(BuiltinCameraDirection.BACK)
                    .build();

            portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));


            waitForStart();
            telemetry.addData("Prop Position", redpropPipeline.getPropPosition());
            telemetry.update();

        }

    }

}
