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
import org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline;
import org.firstinspires.ftc.teamcode.PATCHY.TELEOP.CenterstageTeleop;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


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
    public org.firstinspires.ftc.teamcode.PATCHY.PIPELINES.bluepropPipeline bluepropPipeline;

    public RevBlinkinLedDriver lights;

    Pose2d visPose;
    Pose2d placePose;

    public RevBlinkinLedDriver blinkin;

    int pixelCount;

    //the one of the flap that falls when the bot starts, im not generally sure what it detects tho tbh
    public DigitalChannel flapSns;
    //arm sensor on the bot, the one at the front
    public DigitalChannel armSns;

    //defining the
    public RevBlinkinLedDriver.BlinkinPattern
            SickColor = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES,
            Aqua = RevBlinkinLedDriver.BlinkinPattern.AQUA,
            black = RevBlinkinLedDriver.BlinkinPattern.BLACK,

    blue = RevBlinkinLedDriver.BlinkinPattern.BLUE,
            pink = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;

    public enum robotState {
        intaking,
        idle,
        liftlowering;
    }

    public enum intakeState {
        off,
        intake
    }

    //is the pixel near the proximity sensor boolean control
    public boolean checkFirst;
    public boolean checkSecond;
    public robotState state = robotState.idle;
    public intakeState iState = intakeState.off;

    public hardwareCS() {
        //nothing goes in here, just a way to call the class (stolen from FF hardware map)
    }

    public void inithardware(HardwareMap thisHwMap) {
        hw = thisHwMap;


        mtrBL = hw.get(DcMotorEx.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(BRAKE);
        mtrBL.setDirection(DcMotorEx.Direction.FORWARD);

        mtrBR = hw.get(DcMotorEx.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(BRAKE);
        mtrBR.setDirection(DcMotorEx.Direction.REVERSE);

        mtrFL = hw.get(DcMotorEx.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(BRAKE);
        mtrFL.setDirection(DcMotorEx.Direction.FORWARD);

        mtrFR = hw.get(DcMotorEx.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(BRAKE);
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
        flapSns = hw.get(DigitalChannel.class, "flap");
        armSns = hw.get(DigitalChannel.class, "armSns");

        lights = hw.get(RevBlinkinLedDriver.class, "lights");

        // blinkin = hw.get(RevBlinkinLedDriver.class, "blinkin");

        joggingup = false;
        joggingdown = false;
        mtrLift.setVelocity(0);
        mtrLift2.setVelocity(0);
        droneLauncher.setPosition(1.0);
        armSwing.setPosition(0.1);
        gripper.setPosition(.6);
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

    public String getOutString() {
        return bluepropPipeline.getPropPosition();

    }

    public void LEDcontrol() {

        if (!proximityOne.getState()) {
            checkFirst = true;
        } else {
            checkFirst = false;
        }

        if (!proximityTwo.getState()) {
            checkSecond = true;
        } else {
            checkSecond = false;
        }

        if (checkFirst) {
            if (checkSecond) {
                pixelCount = 2;
            } else {
                pixelCount = 1;
            }
        } else if (checkSecond) {
            pixelCount = 1;
        } else if (!checkFirst && !checkSecond) {
            pixelCount = 0;
        }

        switch (state) {
            case idle:
                //IF WE HAVE NO PIXELS or 1 AND OUR INTAKE ARMS ARE IN THE MIDDLE
                if (intakeLeft.getPosition() == .07 && pixelCount <= 1) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                } else if (intakeLeft.getPosition() == 0 && pixelCount <= 1) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                } else if (checkFirst && checkSecond) {
                    //we got the pixels right?
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
                } else {
                    //IF NONE OF THE OTHER CONDITIONS APPLY
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }

                break;
            case liftlowering:

                if (bottomLimit.isPressed()) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
                } else {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                }
                break;
            default:
                state = robotState.idle;
                break;
        }

    }

    public void intakeControl() {
        armSns.getState();
        flapSns.getState();

        switch (iState) {
            case intake:
                if (checkSecond && checkFirst) {
                    iState = intakeState.off;
                }

                break;
            case off:
                if (!flapSns.getState()) {
                    if (checkSecond && checkFirst) {

                    }
                }
                break;
            default:
                iState = intakeState.off;
        }
    }

    public void toggle(Boolean gamepadbutton, Servo object, double positionGoTo, double positionStartAt) {
        Boolean controlVariable = null;
        if (gamepadbutton && !controlVariable) {
            if (object.getPosition() <= positionStartAt) {
                object.setPosition(positionGoTo);
                controlVariable = true;
            } else {
                object.setPosition(positionStartAt);
                controlVariable = true;
            }
        } else if (!gamepadbutton) {
            controlVariable = false;
        }

    }
}
