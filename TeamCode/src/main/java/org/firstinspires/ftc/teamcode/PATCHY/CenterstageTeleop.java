package org.firstinspires.ftc.teamcode.PATCHY;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardwareCS;
import org.firstinspires.ftc.teamcode.pocCode.IntakeServo;
//


@TeleOp(name = "CenterstageTeleop", group = "Linear Opmode")
//@Disabled
public class CenterstageTeleop extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    //ElapsedTime servoTime = new ElapsedTime();

    //motors
    public DcMotor mtrBL;
    public DcMotor mtrBR;
    public DcMotor mtrFL;
    public DcMotor mtrFR;
    public DcMotor mtrI;
    public DcMotor mtrHang;

    //limit switch
    //TouchSensor limitSwitch;

    //time

    //servos when we get to it
    public Servo intakeLeft;
    public Servo intakeRight;
    public Servo svrHang;

    //copy and paste from LiftCode.java
    public DcMotorEx mtrLift;
    public DcMotorEx mtrLift2;
    public Servo armSwing;
    public Servo gripper;
    public TouchSensor bottomLimit;
    public Servo droneLauncher;
    public Servo DroneShooter;

    //couple variables controlling our lift
    boolean joggingup;
    boolean joggingdown;
    boolean gripperPressed;

    ElapsedTime servoTime = new ElapsedTime();
    IntakeServo.intakeState intakePos = IntakeServo.intakeState.intakeOut;


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

        hardwareCS robot = new hardwareCS();
        robot.inithardware(hardwareMap);

        //start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

             //GAMEPAD 1 CONTROLS

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * (1 - (gamepad1.right_trigger * 0.7)),
                            gamepad1.left_stick_x * (1 - (gamepad1.right_trigger * 0.7)),
                            gamepad1.right_stick_x * (1 - (gamepad1.right_trigger * 0.7))
                    )
            );

            // adding gamepad two controls here fellas
            /*
             Controls:
                Drone Launcher:
                    Left Bumper = Launch Drone
                Intake:
                    Dpad Right = Intake
                    Dpad Left = Outtake
                    B = Kill Power
                    Arm Positions:
                        Y = Intake arms out
                        X = Intake arms middle
                        A = Intake arms in
                Hanger:
                    Gamepad 2:
                        Back = Release the lift
                    Gamepad 2:
                        Dpad Up = Extend hanger
                        Dpad Down = Retract Hanger
                        Neither = Kill Power
                Lift:
                    Right Trigger = Lift move up
                    Left Trigger = Lift move down
                Subcontrols:
                    Left stick = Move arm
                    Left Bumper Bumper = Open gripper
                    !Right Bumper = Close gripper
             */

            //hanger servo
            if (gamepad2.back) {
                svrHang.setPosition(.9);
            }

            //intake
            if (gamepad2.dpad_right) {
                mtrI.setDirection(DcMotor.Direction.REVERSE);
                mtrI.setPower(0.75);
            }
            if (gamepad2.dpad_left) {
                mtrI.setDirection(DcMotor.Direction.FORWARD);
                mtrI.setPower(0.75);
            }
            if (gamepad2.b) {
                mtrI.setPower(0);
            }

            if (gamepad2.y){
                intakeLeft.setPosition(0.25);
                intakeRight.setPosition(.82);
            }

           /* switch (intakePos) {
                case intakeIn:
                    if (gamepad2.y) {
                        intakeRight.setPosition(0.85);
                        intakePos = IntakeServo.intakeState.intakeOut;
                    }
                    if (gamepad2.x) {
                        intakeRight.setPosition(.82);
                        servoTime.reset();
                        intakePos = IntakeServo.intakeState.intakeMiddle;
                    }

                    break;

                case intakeMiddle:
                    if (gamepad2.y) {
                        intakeRight.setPosition(0.85);
                        intakePos = IntakeServo.intakeState.intakeOut;
                    }
                    if (gamepad2.a) {
                        intakeLeft.setPosition(.557);
                        servoTime.reset();
                        intakePos = IntakeServo.intakeState.intakeIn;
                    }

                    break;

                case intakeOut:
                    if (gamepad2.a) {
                        intakeLeft.setPosition(.557);
                        servoTime.reset();
                        intakePos = IntakeServo.intakeState.intakeIn;
                    }
                    if (gamepad2.x) {
                        intakeRight.setPosition(.82);
                        servoTime.reset();
                        intakePos = IntakeServo.intakeState.intakeMiddle;
                    }

                    break;

                default:
                    intakePos = IntakeServo.intakeState.intakeOut;
            } */

            //middle pos
            /*
            if (gamepad1.b) {
                intakeRight.setPosition(.82);
                servoTime.reset();
                lastBtn = 'b';

            }
            //inside pos
            if (gamepad1.y) {
                intakeLeft.setPosition(.557);
                servoTime.reset();
                lastBtn = 'y';

            }

            if (gamepad1.x) {
                intakeRight.setPosition(0.85);
                lastBtn = 'x';
            }

            //after x seconds have passed, what button was last pressed? then set position.
           if (servoTime.time() > .75) {

               if (lastBtn == 'b') {
                   intakeLeft.setPosition(0.25);
               }

               if (lastBtn == 'y') {
                   intakeRight.setPosition(0.672);
               }
               if (lastBtn == 'x') {
                   intakeLeft.setPosition(0.15);
               }

           } */

            //hanging
            if (gamepad2.dpad_down /*&& elapsedTime.time() > 60)*/) {
                mtrHang.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrHang.setPower(1.0);
            }
//up
            if (gamepad2.dpad_up) {
                mtrHang.setDirection(DcMotor.Direction.REVERSE);
                mtrHang.setPower(1.0);

            }

            if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
                mtrHang.setPower(0);
            }

            //slides

            if (gamepad2.right_trigger >= .9) {
                mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift.setVelocity(1000);
                mtrLift2.setVelocity(1000);
                joggingup = true;

            }

            if (gamepad2.right_trigger == 0 && joggingup) {
                mtrLift.setVelocity(0);
                mtrLift2.setVelocity(0);
                joggingup = false;
            } else if (gamepad2.left_trigger >= .9 && !bottomLimit.isPressed()) {
                mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift.setVelocity(1000);
                mtrLift2.setVelocity(1000);
                joggingdown = true;

            }

            if (joggingdown && bottomLimit.isPressed()) {
                mtrLift.setVelocity(0);
                mtrLift2.setVelocity(0);
                joggingdown = false;
            }


            if (gamepad2.left_trigger == 0 && joggingdown) {
                mtrLift.setVelocity(0);
                mtrLift2.setVelocity(0);
                joggingdown = false;
            }

            //drone launcher
            if (gamepad1.left_bumper) {
                droneLauncher.setPosition(.15);
            }

            //when we get there
           /* if (gamepad2.right_bumper) {
                gripper.setPosition(0.5);
            } else if (!gamepad2.right_bumper) {
                gripper.setPosition(0.5);
            }*/

            armSwing.setPosition(armSwing.getPosition() - (gamepad2.left_stick_y * .01));

            if (gamepad2.left_bumper && !gripperPressed) {
                if (!gamepad2.left_bumper) {
                    gripper.setPosition(0.57);
                    gripperPressed = true;
                }
            } else if (gamepad2.left_bumper && gripperPressed){
                if (!gamepad2.left_bumper) {
                    gripper.setPosition(.32);
                    gripperPressed = false;
                }
            }


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x:", poseEstimate.getX());
            telemetry.addData("y:", poseEstimate.getY());
            telemetry.addData("heading:", poseEstimate.getHeading());

            telemetry.update();
            drive.update();

        }
    }

}
