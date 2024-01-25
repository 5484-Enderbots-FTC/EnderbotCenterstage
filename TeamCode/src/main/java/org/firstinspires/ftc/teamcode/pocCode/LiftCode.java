package org.firstinspires.ftc.teamcode.pocCode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
//


@TeleOp(name="Lift Code")
public class LiftCode extends LinearOpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode
    public DcMotorEx mtrLift;
    public DcMotorEx mtrLift2;
    Servo armSwing;
    Servo gripper;

    // the dump servo
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    private int LIFT_LOW = 600; // the low encoder position for the lift
    private int LIFT_HIGH = 700;// the high encoder position for the lift

    private double DUMP_TIME = 1.00;
    @Override
    public void runOpMode() throws InterruptedException {
        liftTimer.reset();

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


        waitForStart();

        while (opModeIsActive()) {
           /* switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad1.x) {
                        // x is pressed, start extending
                        mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                        mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
                        mtrLift.setTargetPosition(LIFT_HIGH);
                        mtrLift2.setTargetPosition(LIFT_HIGH);
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    // check if the lift has finished extending,
                    // otherwise do nothing.
                    if (Math.abs(mtrLift.getCurrentPosition() - LIFT_HIGH) < 20) {
                        // our threshold is within
                        // 20 encoder ticks of our target.
                        // this is pretty arbitrary, and would have to be
                        // tweaked for each robot.
                        // set the lift dump to dump
                        liftTimer.reset();
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                case LIFT_DUMP:
                    if (gamepad1.b) {
                        // The robot waited long enough, time to start
                        // retracting the lift
                        mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                        mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                        mtrLift.setTargetPosition(LIFT_LOW);
                        mtrLift2.setTargetPosition(LIFT_LOW);
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if (Math.abs(mtrLift.getCurrentPosition() - LIFT_LOW) < 10) {
                        liftState = LiftState.LIFT_START;
                    }
                    break;
                default:
                    // should never be reached, as liftState should never be null
                    liftState = LiftState.LIFT_START;
            }

            // small optimization, instead of repeating ourselves in each
            // lift state case besides LIFT_START for the cancel action,
            // it's just handled here
            if (gamepad1.y && liftState != LiftState.LIFT_START) {
                mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift.setTargetPosition(LIFT_LOW);
                mtrLift2.setTargetPosition(LIFT_LOW);
                liftState = LiftState.LIFT_START;
            } */

            // mecanum drive code goes here
            // But since none of the stuff in the switch case stops
            // the robot, this will always run!
            telemetry.addData("What's it doing? (Nothing with this code): ", liftState);
            telemetry.addData("Top motor encoder: ", mtrLift.getCurrentPosition());
            telemetry.addData("Bottom motor encoder: ", mtrLift2.getCurrentPosition());
            telemetry.update();

            if (gamepad1.b) {
                mtrLift.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift2.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift.setTargetPosition(LIFT_HIGH);
                mtrLift2.setTargetPosition(LIFT_HIGH);
                mtrLift.setVelocity(2000);
                mtrLift2.setVelocity(2000);

            }

            if (gamepad1.x){
                mtrLift.setDirection(DcMotorSimple.Direction.FORWARD);
                mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrLift.setTargetPosition(LIFT_LOW);
                mtrLift2.setTargetPosition(LIFT_LOW);
                mtrLift.setVelocity(2000);
                mtrLift2.setVelocity(2000);
            }

            armSwing.setPosition(armSwing.getPosition() - (gamepad1.left_stick_y * .0001));
            gripper.setPosition(gripper.getPosition() - (gamepad1.right_stick_y * .0001));


        }
    }
}



