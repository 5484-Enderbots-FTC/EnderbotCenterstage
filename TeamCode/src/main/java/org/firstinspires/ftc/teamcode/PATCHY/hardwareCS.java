package org.firstinspires.ftc.teamcode.PATCHY;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PATCHY.AUTOS.PIPELINES.bluepropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;


public class hardwareCS {

    public DcMotor mtrFL;
    public DcMotor mtrFR;
    public DcMotor mtrBR;
    public DcMotor mtrBL;

    public DcMotor mtrI;

    public DcMotor mtrHang;

    public HardwareMap hw = null;

    private VisionPortal visionPortal;
    public OpenCvCamera webcam;

    public ColorSensor liftColor;

    public DigitalChannel proximityOne;
    public DigitalChannel proximityTwo;
    public Servo intakeLeft;
    public Servo intakeRight;
    public Servo svrHang;
    public DcMotorEx mtrLift;
    public DcMotorEx mtrLift2;
    public Servo armSwing;
    public Servo gripper;
    public TouchSensor bottomLimit;
    public Servo droneLauncher;
    public Servo DroneShooter;
    boolean joggingup;
    boolean joggingdown;
    boolean gripperPressed;

    //auto things
    public String auto;
    public VisionPortal portal;
    public org.firstinspires.ftc.teamcode.PATCHY.AUTOS.PIPELINES.bluepropPipeline bluepropPipeline;

    Pose2d visPose;
    Pose2d placePose;

    public hardwareCS() {
        //nothing goes in here, just a way to call the class (stolen from FF hardware map)
    }

    public void inithardware(HardwareMap thisHwMap) {
        hw = thisHwMap;

        mtrBL = hw.get(DcMotorEx.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotorEx.Direction.FORWARD);

        mtrBR = hw.get(DcMotorEx.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotorEx.Direction.REVERSE);

        mtrFL = hw.get(DcMotorEx.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFR = hw.get(DcMotorEx.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotorEx.Direction.REVERSE);

        mtrI = hw.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.REVERSE);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrHang = hw.get(DcMotor.class, "mtrHang");
        mtrHang.setZeroPowerBehavior(BRAKE);
        mtrHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrHang.setDirection(DcMotor.Direction.FORWARD);

        droneLauncher = hw.get(Servo.class, "svrDrone");

        mtrLift = hw.get(DcMotorEx.class, "mtrLift1");
        mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLift.setZeroPowerBehavior(BRAKE);

        mtrLift2 = hw.get(DcMotorEx.class, "mtrLift2");
        mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLift2.setZeroPowerBehavior(BRAKE);

        gripper = hw.get(Servo.class, "svrGrip");
        armSwing = hw.get(Servo.class, "svrSwing");

        bottomLimit = hw.get(TouchSensor.class, "BL Lift");

        liftColor = hw.get(ColorSensor.class, "clrSensor");

        intakeLeft = hw.get(Servo.class, "leftSvr");
        intakeRight = hw.get(Servo.class, "rightSvr");
        svrHang = hw.get(Servo.class, "svrHang");
        svrHang.setPosition(0.53);

        //intakeRight.setPosition(0.94);
        //intakeLeft.setPosition(0.027);

        proximityOne = hw.get(DigitalChannel.class, "nut1");
        proximityTwo = hw.get(DigitalChannel.class, "nut2");

        joggingup = false;
        joggingdown = false;
        mtrLift.setVelocity(0);
        mtrLift2.setVelocity(0);
        droneLauncher.setPosition(1.0);
        gripperPressed = false;

        // set digital channel to input mode.
        proximityOne.setMode(DigitalChannel.Mode.INPUT);
        proximityTwo.setMode(DigitalChannel.Mode.INPUT);


    }


    public void initWebcamBlue() {
        bluepropPipeline = new bluepropPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(bluepropPipeline)
                .build();

        //portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d"));
        //if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            //telemetry.addData("Camera", "Waiting");
           //telemetry.update();
            /*while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }*/
            //telemetry.addData("Camera", "Ready");
            //telemetry.update();
        //}
        //prop position for autonomous, auto = prop position
        //auto = bluepropPipeline.getPropPosition();
        //telemetry.addData("blue Prop Position", bluepropPipeline.getPropPosition());
        //telemetry.update();
    }

    public String getOutString(){
        return bluepropPipeline.getPropPosition();

    }


}
