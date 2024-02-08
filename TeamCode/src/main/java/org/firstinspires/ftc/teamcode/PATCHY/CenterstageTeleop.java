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

    public double gamepadlsy;

    public double gamepadlsx;

    public int n = 0;

    public int m = 0;

    public boolean currentlyPressing = false;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Really Chat :|");
        telemetry.addData("" +
        "              ⠀⠀⠘⡀⠀⠀⠀have a nice day nerd!⡜⠀⠀⠀\n" +
                "             * ⠀⠀⠀⠑⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡔⠁⠀⠀⠀\n" +
                "             * ⠀⠀⠀⠀⠈⠢⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠴⠊⠀⠀⠀⠀⠀\n" +
                "             * ⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⢀⣀⣀⣀⣀⣀⡀⠤⠄⠒⠈⠀⠀⠀⠀⠀⠀⠀⠀\n" +
                "             * ⠀⠀⠀⠀⠀⠀⠀⠘⣀⠄⠊⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n" +
                "             * ⠀\n" +
                "             * ⣿⣿⣿⣿⣿⣿⣿⣿⡿⠿⠛⠛⠛⠋⠉⠈⠉⠉⠉⠉⠛⠻⢿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣿⡿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⢿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⡏⣀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣤⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠙⢿⣿⣿\n" +
                "             * ⣿⣿⣿⢏⣴⣿⣷⠀⠀⠀⠀⠀⢾⣿⣿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⠀⠈⣿⣿\n" +
                "             * ⣿⣿⣟⣾⣿⡟⠁⠀⠀⠀⠀⠀⢀⣾⣿⣿⣿⣿⣿⣷⢢⠀⠀⠀⠀⠀⠀⠀⢸⣿\n" +
                "             * ⣿⣿⣿⣿⣟⠀⡴⠄⠀⠀⠀⠀⠀⠀⠙⠻⣿⣿⣿⣿⣷⣄⠀⠀⠀⠀⠀⠀⠀⣿\n" +
                "             * ⣿⣿⣿⠟⠻⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠶⢴⣿⣿⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⣿\n" +
                "             * ⣿⣁⡀⠀⠀⢰⢠⣦⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⣿⣿⣿⣿⣿⡄⠀⣴⣶⣿⡄⣿\n" +
                "             * ⣿⡋⠀⠀⠀⠎⢸⣿⡆⠀⠀⠀⠀⠀⠀⣴⣿⣿⣿⣿⣿⣿⣿⠗⢘⣿⣟⠛⠿⣼\n" +
                "             * ⣿⣿⠋⢀⡌⢰⣿⡿⢿⡀⠀⠀⠀⠀⠀⠙⠿⣿⣿⣿⣿⣿⡇⠀⢸⣿⣿⣧⢀⣼\n" +
                "             * ⣿⣿⣷⢻⠄⠘⠛⠋⠛⠃⠀⠀⠀⠀⠀⢿⣧⠈⠉⠙⠛⠋⠀⠀⠀⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣧⠀⠈⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠟⠀⠀⠀⠀⢀⢃⠀⠀⢸⣿⣿⣿⣿\n" +
                "             * ⣿⣿⡿⠀⠴⢗⣠⣤⣴⡶⠶⠖⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡸⠀⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⡀⢠⣾⣿⠏⠀⠠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⠉⠀⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣧⠈⢹⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⡄⠈⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣴⣾⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣿⣦⣄⣀⣀⣀⣀⠀⠀⠀⠀⠘⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⡄⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⠀⠀⠀⠙⣿⣿⡟⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠇⠀⠁⠀⠀⠹⣿⠃⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⣿⣿⣿⣿⡿⠛⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢐⣿⣿⣿⣿⣿⣿⣿⣿⣿\n" +
                "             * ⣿⣿⣿⣿⠿⠛⠉⠉⠁⠀⢻⣿⡇⠀⠀⠀⠀⠀⠀⢀⠈⣿⣿⡿⠉⠛⠛⠛⠉⠉\n" +
                "             * ⣿⡿⠋⠁⠀⠀⢀⣀⣠⡴⣸⣿⣇⡄⠀⠀⠀⠀⢀⡿⠄⠙⠛⠀⣀⣠⣤⣤⠄\n",0);
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
        robot.armSwing.setPosition(.25);

        //start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

             //GAMEPAD 1 CONTROLS
            //set a new pose based off our previous pose utilizing the positions of gamepad sticks
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
            //release the little mechanism holding the hanger mechanism down
            if (gamepad2.back) {
                robot.svrHang.setPosition(.9);
            }

            //intake
            //take in pixels
            if (gamepad2.dpad_right) {
                robot.mtrI.setDirection(DcMotor.Direction.REVERSE);
                robot.mtrI.setPower(0.75);
            }
            //spit out pixels
            if (gamepad2.dpad_left) {
                robot.mtrI.setDirection(DcMotor.Direction.FORWARD);
                robot.mtrI.setPower(0.75);
            }
            //kill the power on the intake
            if (gamepad2.b) {
                robot.mtrI.setPower(0);
            }

            if (gamepad2.y){
                robot.intakeLeft.setPosition(0.25);
                robot.intakeRight.setPosition(.82);
            }
            
           /* if (gamepad2.y){
                intakeLeft.setPosition(0.25);
                intakeRight.setPosition(.82);
            } */
//this *was* a state machine to control the position of the servo arms
//one day i will fix this, but i can make it work without it. Less code readability,
//but i can at least make it function
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
            //extend the lift
            //currently, the timer to only make it activate during endgame
            //is commented out b/c katarina wanted it to be 
            if (gamepad2.dpad_down /*&& elapsedTime.time() > 60)*/) {
                robot.mtrHang.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.mtrHang.setPower(1.0);
            }
//up         
            //beam me up, Scotty
            if (gamepad2.dpad_up) {
                robot.mtrHang.setDirection(DcMotor.Direction.REVERSE);
                robot.mtrHang.setPower(1.0);

            }
            //straightforward, if we're pressing neither then
            //stop the hanger
            if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
                robot.mtrHang.setPower(0);
            }

            //slides control

            //if pressing the right trigger, move up, and
            //set our memory bit to true.

            //motors were connected electrically, so we don't need to control
            //mtrLift2. However, it makes no difference whether we do or not.

            if (gamepad2.right_trigger >= .9) {
                robot.mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.mtrLift.setVelocity(1000);
                robot.mtrLift2.setVelocity(1000);
                joggingup = true;

            }
            //if we were already jogging up, then stop the lift.
            //we need this extra variable check b/c otherwise
            //gamepad2 right trigger will be false while we're moving
            //the lift downward, which will constantly stop our lift
            //and not let us move down.
            if (gamepad2.right_trigger == 0 && joggingup) {
                robot.mtrLift.setVelocity(0);
                robot.mtrLift2.setVelocity(0);
                joggingup = false;

            } else if (gamepad2.left_trigger >= .9 && !robot.bottomLimit.isPressed()) {
                robot.mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.mtrLift.setVelocity(1000);
                robot.mtrLift2.setVelocity(1000);
                joggingdown = true;

            }
            //if we're moving down but our limit switch pressed, STOP!!!!
            if (joggingdown && robot.bottomLimit.isPressed()) {
                robot.mtrLift.setVelocity(0);
                robot.mtrLift2.setVelocity(0);
                joggingdown = false;
                
            //if we're holding down the left trigger & our limit switch isn't pressed
            //move our lift down, and set our memory/control bit to true.
            } else if (gamepad2.left_trigger >= .9 && !bottomLimit.isPressed()) {
                mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift.setVelocity(1000);
                mtrLift2.setVelocity(1000);
                joggingdown = true;

            }

            //if we stop holding down our trigger and
            //we were moving down to begin with,
            //stop the lift.
            //this extra variable check is in place b/c of the
            //same reasoning as the other trigger check
            //this also doesn't need the limit switch to stop
            if (gamepad2.left_trigger == 0 && joggingdown) {
                robot.mtrLift.setVelocity(0);
                robot.mtrLift2.setVelocity(0);
                joggingdown = false;
            }

            //shoot the drone launcher
            if (gamepad1.left_bumper) {
                robot.droneLauncher.setPosition(.15);
            }

            robot.armSwing.setPosition(robot.armSwing.getPosition() - (gamepad2.left_stick_y * .01));

            //if we are pressing the left bumper, weren't already open, and weren't holding
            //the button to begin with, then open the gripper to holding position.
            if (gamepad2.left_bumper && !gripperPressed && !currentlyPressing) {
                    gripper.setPosition(0.57);

            //if we are pressing the left bumper, were already open, and weren't holding
            //the button to begin with, the close gripper to placement position
            } else if (gamepad2.left_bumper && gripperPressed && !currentlyPressing){
                    gripper.setPosition(.32);
            }

            //variable control telling us where the servo is. Is it open or closed? Change variable based on that.
            if (gripper.getPosition() > .56) {
                gripperPressed = true;
            } else if (gripper.getPosition() < .33) {
                gripperPressed = false;
            }

            //are we currently pressing the left bumper? if so, we can't change the position of our gripper until we do.
            if (gamepad2.left_bumper) {
                currentlyPressing = true;
            } else {
                currentlyPressing = false;
            }

//this is just adding our data to our code.

            //where are we?
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x:", poseEstimate.getX());
            telemetry.addData("y:", poseEstimate.getY());
            telemetry.addData("heading:", poseEstimate.getHeading());

            //is our gripper working?
            if (gripper.getPosition() > .56) {
                telemetry.addLine("Gripper Position: Pixels being held.");
            } else {
                telemetry.addLine("Gripper Position: Not grabbing pixels.");
            }

            telemetry.update();
            drive.update();

         }
    }

}
