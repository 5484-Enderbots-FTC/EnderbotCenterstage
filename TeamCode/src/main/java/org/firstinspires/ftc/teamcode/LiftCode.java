package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//


@TeleOp(name = "LiftCode", group = "Linear Opmode")
//@Disabled
public class LiftCode extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Servo grab;
    double armOffset = 0;
    Servo arm;
    DcMotorEx mtrLift;
    TouchSensor lowLimit;

    int LiftOffset = 0;

    int floor = 0;
    boolean floorBool = true;
    int firstHeight = 1600;

    boolean firstHeightBool = false;
    int secondHeight = 3300;
    boolean secondHeightBool = false;
    int fullHeight = 5250;
    boolean fullHeightBool = false;
    boolean neutralHoldBool = false;
    boolean lowGrab = false;
    boolean highGrab = false;
    //servos
    boolean armOutButton = false;
    boolean gripCloseButton = false;
    boolean armOut = false;
    boolean gripClose = false;
    int PlacingMode = 1;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mtrLift = hardwareMap.get(DcMotorEx.class, "mtrLift");
        mtrLift.setZeroPowerBehavior(BRAKE);
        mtrLift.setDirection(DcMotor.Direction.REVERSE);
        mtrLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lowLimit = hardwareMap.get(TouchSensor.class, "lowLimit");

        while (opModeIsActive()) {


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("mtrLift", "whee " + mtrLift.getCurrentPosition());
            telemetry.addData("limit", "whee " + lowLimit.isPressed());

            if (lowLimit.isPressed()) {
                telemetry.addData("skreeblo", lowLimit.isPressed());
            } else {
                telemetry.addData("skwimble", lowLimit.isPressed());
            }

            if (gamepad2.x && gripCloseButton == false && gripClose == false) {
                gripClose = true;
                gripCloseButton = true;

            }
            if (gamepad2.x && gripCloseButton == false && gripClose == true) {
                gripClose = false;
                gripCloseButton = true;

            }
            }

            if (gamepad2.b) {
                LiftOffset = 0;
            }

            LiftOffset = (int) (LiftOffset - (gamepad2.left_stick_y * 4));
            //grab.setPosition(gamepad2.right_trigger);

            if (mtrLift.getCurrentPosition() <= 4500 && armOut == true) {
                grab.setPosition(.94 - armOffset);
            } else if (mtrLift.getCurrentPosition() > 4500 && armOut == true) {
                grab.setPosition(.94 - armOffset);
            } else if (armOut == false) {
                grab.setPosition(.15 - armOffset);
            }

            if (armOut == false) {
                PlacingMode = 1;
            } else {
                PlacingMode = -1;
            }

            if (!gamepad2.x) {
                gripCloseButton = false;
            }
            if (!gamepad2.a) {
                armOutButton = false;
            }

            telemetry.addData("lowgrab", lowGrab);
            telemetry.addData("highgrab", highGrab);

            if (gamepad2.dpad_left) {
                floorBool = false;
                firstHeightBool = true;
                secondHeightBool = false;
                fullHeightBool = false;
                neutralHoldBool = false;
            }
            if (gamepad2.dpad_down) {
                floorBool = true;
                firstHeightBool = false;
                secondHeightBool = false;
                fullHeightBool = false;
                neutralHoldBool = false;
            }
            if (gamepad2.dpad_right) {
                floorBool = false;
                firstHeightBool = false;
                secondHeightBool = false;
                fullHeightBool = true;
                neutralHoldBool = false;
            }
            if (gamepad2.dpad_up) {
                floorBool = false;
                firstHeightBool = false;
                secondHeightBool = true;
                fullHeightBool = false;
                neutralHoldBool = false;
            }

            if (gamepad2.a) {
                floorBool = false;
                firstHeightBool = false;
                secondHeightBool = false;
                fullHeightBool = false;
                neutralHoldBool = true;
            }

            if (floorBool) {
                mtrLift.setTargetPosition(floor + LiftOffset);
                mtrLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mtrLift.setVelocity(5000);
                armOut = false;
            }

            if (neutralHoldBool) {
                mtrLift.setTargetPosition(floor + LiftOffset);
                mtrLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mtrLift.setVelocity(5000);
                armOut = true;
            }

            if (firstHeightBool) {
                mtrLift.setTargetPosition(firstHeight + LiftOffset);
                mtrLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mtrLift.setVelocity(5000);
                armOut = true;
            }
            if (secondHeightBool) {
                mtrLift.setTargetPosition(secondHeight + LiftOffset);
                mtrLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mtrLift.setVelocity(5000);
                armOut = true;
            }
            if (fullHeightBool) {
                mtrLift.setTargetPosition(fullHeight + LiftOffset);
                mtrLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mtrLift.setVelocity(5000);
                armOut = true;
            }

            telemetry.addData("floorGoal", floorBool);
            telemetry.addData("firstGoal", firstHeightBool);
            telemetry.addData("secondGoal", secondHeightBool);
            telemetry.addData("floorGoal", fullHeightBool);
            telemetry.update();
            drive.update();


        }
    }




