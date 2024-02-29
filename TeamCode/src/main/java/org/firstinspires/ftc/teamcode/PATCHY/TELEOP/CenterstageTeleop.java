package org.firstinspires.ftc.teamcode.PATCHY.TELEOP;

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

import org.firstinspires.ftc.teamcode.PATCHY.hardwareCS;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PATCHY.pocCode.INTAKE.IntakeServo;
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
    boolean joggingup = false;
    boolean joggingdown = false;
    boolean gripperPressed;

    ElapsedTime servoTime = new ElapsedTime();
    IntakeServo.intakeState intakePos = IntakeServo.intakeState.intakeOut;

    public double gamepadlsy;

    public double gamepadlsx;

    public int n = 0;

    public int m = 0;

    public boolean currentlyPressing = false;


    //toggle variables for the various toggle buttons
    public int gripperInt = 0;
    public boolean oldGripperInt = false;
    public int intakeArmInt = 0;
    public boolean oldIntakeArmInt = false;
    public int armInt = 0;
    public boolean oldArmInt = false;


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
        robot.armSwing.setPosition(0.0);
        robot.gripper.setPosition(.37);

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
                        Y = Toggle intake arms
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
                    A = Toggle different arm positions
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

//outside
            /*if (gamepad2.x && intakeArmInt == 0 && !oldIntakeArmInt) {
                robot.intakeLeft.setPosition(0);
                robot.intakeRight.setPosition(1);
                intakeArmInt = 1;
            }
            //middle
            if (gamepad2.x && intakeArmInt == 1 && !oldIntakeArmInt) {
                robot.intakeRight.setPosition(0.92);
                robot.intakeLeft.setPosition(.07);
                intakeArmInt = 0;
            }*/

            if(gamepad2.x && !oldIntakeArmInt) {
                if  (robot.intakeLeft.getPosition() == 0 && robot.intakeRight.getPosition() == 1) {
                    robot.intakeLeft.setPosition(.07);
                    robot.intakeRight.setPosition(.92);
                    oldIntakeArmInt = true;
                } else {
                    robot.intakeRight.setPosition(1);
                    robot.intakeLeft.setPosition(0);
                    oldIntakeArmInt = true;
                }
            } else if(!gamepad2.x) {
                oldIntakeArmInt = false;
            }

            //control and make sure we can't activate the above functions unless we aren't pressing the x button


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

            if (gamepad2.right_trigger >= .9 && !joggingdown) {
                robot.mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                //robot.mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.mtrLift.setVelocity(1000);
                //robot.mtrLift2.setVelocity(1000);
                joggingup = true;

            }
            //if we were already jogging up, then stop the lift.
            //we need this extra variable check b/c otherwise
            //gamepad2 right trigger will be false while we're moving
            //the lift downward, which will constantly stop our lift
            //and not let us move down.
            if (gamepad2.right_trigger == 0 && joggingup) {
                robot.mtrLift.setVelocity(0);
                //robot.mtrLift2.setVelocity(0);
                joggingup = false;
                joggingdown = false;

            }

            if (gamepad2.left_trigger >= .9 && !robot.bottomLimit.isPressed() && !joggingup) {
                robot.mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                //robot.mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.mtrLift.setVelocity(700);
                //robot.mtrLift2.setVelocity(1000);
                joggingdown = true;

            }
            //if we're moving down but our limit switch pressed, STOP!!!!
            if (joggingdown && robot.bottomLimit.isPressed()) {
                robot.mtrLift.setVelocity(0);
                //robot.mtrLift2.setVelocity(0);
                joggingdown = false;
                joggingup = false;
                
            //if we're holding down the left trigger & our limit switch isn't pressed
            //move our lift down, and set our memory/control bit to true.
            }

            //if we stop holding down our trigger and
            //we were moving down to begin with,
            //stop the lift.
            //this extra variable check is in place b/c of the
            //same reasoning as the other trigger check
            //this also doesn't need the limit switch to stop
            if (gamepad2.left_trigger == 0 && joggingdown) {
                robot.mtrLift.setVelocity(0);
                //robot.mtrLift2.setVelocity(0);
                joggingdown = false;
                joggingup = false;
            }

            //shoot the drone launcher
            if (gamepad2.left_bumper) {
                robot.droneLauncher.setPosition(.689);
            }

            //control the arm swing positions
            /*if (gamepad2.a && armInt == 0 && oldArmInt == 0) {
                robot.armSwing.setPosition(1.0);
                oldArmInt = 1;
                armInt = 1;
            }
            if (gamepad2.a && armInt == 1 && oldArmInt == 0) {
                robot.armSwing.setPosition(0.0);
                oldArmInt = 1;
                armInt = 0;
            }

            if (!gamepad1.a && oldArmInt != 0) {
                oldArmInt = 0;
            } else if (gamepad1.a && oldArmInt != 1) {
                oldArmInt = 1;
            }*/

            if(gamepad2.a && !oldArmInt) {
                if  (robot.armSwing.getPosition() == 0) {
                    robot.armSwing.setPosition(1);
                    oldArmInt = true;
                } else {
                    robot.armSwing.setPosition(0);
                    oldArmInt = true;
                }
            } else if(!gamepad2.a) {
                oldArmInt = false;
            }

            //if we are pressing the left bumper, weren't already open, and weren't holding
           /* if (gamepad2.right_bumper && gripperInt == 0 && oldGripperInt == 0) {
                robot.gripper.setPosition(.57);
                gripperInt = 1;
                oldGripperInt = 1;
            }

            if (gamepad2.right_bumper && gripperInt == 1 && oldGripperInt == 0) {
                robot.gripper.setPosition(.32);
                gripperInt = 0;
                oldGripperInt = 1;
            }

            if (!gamepad2.right_bumper && oldGripperInt != 0) {
                oldGripperInt = 0;
            }*/

            if(gamepad2.right_bumper && !oldGripperInt) {
                if  (robot.gripper.getPosition() <=.32) {
                    robot.gripper.setPosition(.57);
                    oldGripperInt = true;
                } else {
                    robot.gripper.setPosition(.32);
                    oldGripperInt = true;
                }
            } else if(!gamepad2.right_bumper) {
                oldGripperInt = false;
            }

            //hardwareCS file, controls our LEDs
            if (gamepad2.left_trigger >= .9){
                robot.state = hardwareCS.robotState.liftlowering;
            }else {
                robot.state = hardwareCS.robotState.idle;
            }
            
            robot.LEDcontrol();



//this is just adding our data to our code.

            //where are we?
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x:", poseEstimate.getX());
            telemetry.addData("y:", poseEstimate.getY());
            telemetry.addData("heading:", poseEstimate.getHeading());

            //is our gripper working?
            if (robot.gripper.getPosition() > .56) {
                telemetry.addLine("Gripper Position: Pixels being held.");
            } else {
                telemetry.addLine("Gripper Position: Not grabbing pixels.");
            }

            if (!robot.proximityOne.getState()) {
                telemetry.addLine("Proximity 1 detecting.");
            } else {
                telemetry.addLine("Proximity 1 not detecting.");
            }

            if (!robot.proximityTwo.getState()) {
                telemetry.addLine("Proximity 2 detecting.");
            } else {
                telemetry.addLine("Proximity 2 not detecting.");
            }

            telemetry.update();
            drive.update();

         }
    }

}
