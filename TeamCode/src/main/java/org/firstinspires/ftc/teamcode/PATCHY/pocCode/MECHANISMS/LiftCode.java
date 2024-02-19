package org.firstinspires.ftc.teamcode.PATCHY.pocCode.MECHANISMS;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
//


@TeleOp(name="Lift Code")
@Disabled
public class LiftCode extends LinearOpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    /*public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }*/

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    //LiftState liftState = LiftState.LIFT_START;

    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode
    public DcMotorEx mtrLift;
    public DcMotorEx mtrLift2;
    Servo armSwing;
    Servo gripper;
    TouchSensor bottomLimit;
    Servo droneLauncher;
    int state = 0;
    int Lift_State = 0;
    boolean gripperPressed;

    // the dump servo
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    private boolean joggingup;
    private boolean joggingdown;

    private double DUMP_TIME = 1.00;
    @Override
    public void runOpMode() throws InterruptedException {
        liftTimer.reset();

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
        gripperPressed = false;
        mtrLift.setVelocity(0);
        mtrLift2.setVelocity(0);
        droneLauncher.setPosition(0.0);
        gripper.setPosition(0.37);

        waitForStart();

        while (opModeIsActive()) {
            switch (state){
                case (0):
                    telemetry.addData("case: ", state);
                    //sleep(1000);
                    state = 10;

                    break;
                case(10):
                    telemetry.addData("case: ", state);
                    //sleep(1000);
                    state = 20;

                    break;
                case(20):
                    telemetry.addData("case: ", state);
                    //sleep(1000);
                    state = 0;

                    break;
                default:
                    state = 0;

            }
//this is the state machine to control the lift
            /*switch (Lift_State){
                case (0):
                    //sleep(1000);
                    if (gamepad1.a) {
                        mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                        mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
                        Lift_State = 10;
                        telemetry.addLine("Gamepad A pressed.");

                    }

                    break;
                case(10):
                    //sleep(1000);
                    //mtrLift.setTargetPosition(LIFT_HIGH);
                    //mtrLift.setVelocity(1000);
                   // mtrLift2.setVelocity(1000);
                   // if (Math.abs(mtrLift.getCurrentPosition() - LIFT_HIGH) >= 10) {

                    mtrLift.setVelocity(100);
                    mtrLift2.setVelocity(100);
                    liftTimer.reset();
                    Lift_State = 20;

                   // }

                    break;
                case(20):
                    //sleep(1000);
                    //if (Math.abs(mtrLift.getCurrentPosition() - LIFT_HIGH) >= 10) {
                        telemetry.addData("motor 1 pos: ", mtrLift.getCurrentPosition());
                    if (liftTimer.time() >= 3) {
                        mtrLift.setVelocity(0);
                        mtrLift2.setVelocity(0);
                        Lift_State = 30;
                    }
                    //}

                    break;
                case(30):
                    if (gamepad1.b) {
                        mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                        mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                        liftTimer.reset();
                        Lift_State = 40;
                    }

                    break;

                case(40):
                    //sleep(1000);

                    mtrLift.setVelocity(1000);
                    mtrLift2.setVelocity(1000);

                    //if (Math.abs(mtrLift.getCurrentPosition() - LIFT_LOW) <= 10) {
                    if (liftTimer.time() > 3) {
                        mtrLift.setVelocity(0);
                        mtrLift2.setVelocity(0);
                        Lift_State = 0;
                    }
                    //}
                    break;
                default:
                    Lift_State = 0;

            }*/

            if (gamepad1.y && Lift_State != 0) {
                mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                Lift_State = 40;

            }

            if (gamepad1.right_trigger >= .9) {
                mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift.setVelocity(700);
                mtrLift2.setVelocity(700);
                joggingup = true;

            }


            if (gamepad1.right_trigger == 0 && joggingup == true) {
                mtrLift.setVelocity(0);
                mtrLift2.setVelocity(0);
                joggingup = false;
            } else if (gamepad1.left_trigger >= .9 && !bottomLimit.isPressed()) {
                mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift.setVelocity(700);
                mtrLift2.setVelocity(700);
                joggingdown = true;

            }

            if (joggingdown == true && bottomLimit.isPressed()) {
                mtrLift.setVelocity(0);
                mtrLift2.setVelocity(0);
                joggingdown = false;
            }


            if (gamepad1.left_trigger == 0 && joggingdown == true) {
                mtrLift.setVelocity(0);
                mtrLift2.setVelocity(0);
                joggingdown = false;
            }

            if (gamepad1.a) {
                droneLauncher.setPosition(.15);
            }

            telemetry.addData("Top motor encoder: ", mtrLift.getCurrentPosition());
            //telemetry.addData("Lcase: ", Lift_State);
            telemetry.addData("Jogging?: ", joggingup);
            telemetry.addData("Drone servo pos: ", droneLauncher.getPosition());
            telemetry.addData("Gripper servo pos: ", gripper.getPosition());
            telemetry.addData("Arm servo pos: ", armSwing.getPosition());
            telemetry.addData("Limit Switch: ", bottomLimit.isPressed());
            telemetry.update();

            if (!gamepad1.a) {
                armSwing.setPosition(armSwing.getPosition() - (gamepad1.left_stick_y * .01));
                gripper.setPosition(gripper.getPosition() - (gamepad1.right_stick_y * .001));
            }
            telemetry.update();

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

        }
    }
}



