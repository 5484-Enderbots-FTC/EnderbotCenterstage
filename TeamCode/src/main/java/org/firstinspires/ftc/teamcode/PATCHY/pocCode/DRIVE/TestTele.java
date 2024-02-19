package org.firstinspires.ftc.teamcode.PATCHY.pocCode.DRIVE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
//


@TeleOp(name = "Test Tele", group = "Linear Opmode")
@Disabled
public class TestTele extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    //ElapsedTime servoTime = new ElapsedTime();

    //motors
    public DcMotor mtrBL;
    public DcMotor mtrBR;
    public DcMotor mtrFL;
    public DcMotor mtrFR;


    //copy and paste from LiftCode.jav

    //couple variables controlling our lift

    public double gamepadlsy;

    public double gamepadlsx;

    public int n = 0;

    public int m = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Really Chat :|");
        telemetry.addLine("Get your TeleOp! Hot and fresh!");
        telemetry.update();
        // motors
        /*mtrBL = hardwareMap.get(DcMotor.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(BRAKE);
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrBR = hardwareMap.get(DcMotor.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(BRAKE);
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrFL = hardwareMap.get(DcMotor.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(BRAKE);
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrFR = hardwareMap.get(DcMotor.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(BRAKE);
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrI =  hardwareMap.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.REVERSE);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtrHang = hardwareMap.get(DcMotor.class, "mtrHang");
        mtrHang.setZeroPowerBehavior(BRAKE);
        mtrHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrHang.setDirection(DcMotor.Direction.FORWARD);

        droneLauncher = hardwareMap.get(Servo.class, "svrDrone");

        // hardware initialization code goes here
        // this needs to correspond with the configuration used
        mtrLift = hardwareMap.get(DcMotorEx.class, "mtrLift1");
        mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtrLift2 = hardwareMap.get(DcMotorEx.class, "mtrLift2");
        mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripper = hardwareMap.get(Servo.class, "svrGrip");
        armSwing = hardwareMap.get(Servo.class, "svrSwing");

        bottomLimit = hardwareMap.get(TouchSensor.class, "BL Lift");

        joggingup = false;
        joggingdown = false;
        mtrLift.setVelocity(0);
        mtrLift2.setVelocity(0);
        droneLauncher.setPosition(0.0);
        gripperPressed = false;

        //servos
        intakeLeft = hardwareMap.get(Servo.class, "leftSvr");
        intakeRight = hardwareMap.get(Servo.class, "rightSvr");
        svrHang = hardwareMap.get(Servo.class, "svrHang");
        svrHang.setPosition(0.53);

        intakeRight.setPosition(1.0);
        intakeLeft.setPosition(0.027);

        DroneShooter = hardwareMap.get(Servo.class, "svrDrone"); */

        //start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //GAMEPAD 1 CONTROLS
            //if statements controlling the gamepadlsy variable, allows us to ramp us and accel along the robot's x
            if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < .1) {
                if (gamepad1.left_stick_y > .1) {
                    gamepadlsy = .1 + ((.01 * n)/2);
                }

                if (gamepad1.left_stick_y < -.1) {
                    gamepadlsy = -.1 - ((.01 * n)/2);
                }

                n += 1;

                if (n >= 1000) {
                    n = 1000;
                }


            }
//if statements controlling the gamepadlsx variable, allows us to ramp us and accel along the robot's y
            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < .1) {
                if (gamepad1.left_stick_x > .1) {
                    gamepadlsx = .1 + ((.01 * m)/2);
                }

                if (gamepad1.left_stick_x < -.1) {
                    gamepadlsx = -1 - ((.01 * m)/2);
                }

                m += 1;

                if (m >= 1000) {
                    m = 1000;
                }


            }

            if (gamepad1.left_stick_x == 0) {
                m -= 100;
                if (m < 0) {
                    m = 0;
                }

                if (m > 5) {
                    gamepadlsx = .1 + ((.01 * m) / 2);
                } else {
                    gamepadlsx = 0;
                }
            }

            if (gamepad1.left_stick_y == 0) {
                n -=100;
                if (n < 0) {
                    n = 0;
                }
                if (n > 5) {
                    gamepadlsy = .1 + ((.01 * n) / 2);
                } else {
                    gamepadlsy = 0;
                }
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepadlsy * (1 - (gamepad1.right_trigger * 0.7)),
                            gamepadlsx * (1 - (gamepad1.right_trigger * 0.7)),
                            gamepad1.right_stick_x * (1 - (gamepad1.right_trigger * 0.7))
                    )
            );
        }
    }
}