package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//


@TeleOp(name = "CenterstageTeleop", group = "Linear Opmode")
//@Disabled
public class CenterstageTeleop extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime servoTime = new ElapsedTime();

    //motors
    DcMotor mtrBL;
    DcMotor mtrBR;
    DcMotor mtrFL;
    DcMotor mtrFR;
    DcMotor mtrI;
    DcMotor mtrHang;

    //limit switch
    TouchSensor limitSwitch;

    //time

    //servos when we get to it
    Servo intakeLeft;
    Servo intakeRight;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // motors

        mtrBL = hardwareMap.get(DcMotor.class, "mtrBL");
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

        //servos
        intakeLeft = hardwareMap.get(Servo.class, "leftSvr");
        intakeRight = hardwareMap.get(Servo.class, "rightSvr");

        telemetry.addLine("Have fun!");
        telemetry.addLine("Get your TeleOp! Hot and fresh!");


        //start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

             //GAMEPAD 1 CONTROLS

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * (1 - (gamepad1.right_trigger * 0.6)),
                            gamepad1.left_stick_x * .8 * (1 - (gamepad1.right_trigger * 0.6)),
                            gamepad1.right_stick_x * (1 - (gamepad1.right_trigger * 0.6))
                    )
            );

            // adding gamepad two controls here fellas

            if (gamepad2.x) {
                mtrI.setDirection(DcMotor.Direction.REVERSE);
                mtrI.setPower(0.75);
            }
            if (gamepad2.dpad_down) {
                mtrI.setDirection(DcMotor.Direction.FORWARD);
                mtrI.setPower(0.75);
            }
            if (gamepad2.dpad_up) {
                mtrI.setPower(0);
            }

            //hanging
            if (gamepad2.a) { //&& !limitSwitch.isPressed() && runtime.time() > 60) { <- comment back in when ready
                mtrHang.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrHang.setPower(0.3);

            } else if (gamepad2.b) {//&& limitSwitch.isPressed() && runtime.time() > 60) { <- comment back in when ready
                mtrHang.setDirection(DcMotor.Direction.FORWARD);
                mtrHang.setZeroPowerBehavior(BRAKE);
                mtrHang.setPower(0.5);
            } else if (gamepad2.y) {
                mtrHang.setPower(0);
            }

            //intake servos
           /**if (gamepad2.right_trigger == 1) {

                intakeRight.setPosition(0.5);
                servoTime.reset();
                if (servoTime.time() > 0.5) {

                    intakeLeft.setPosition(0.0);
                }

            }  else if (gamepad2.right_trigger == 0){

                intakeLeft.setPosition(0.5);
                intakeRight.setPosition(0.0);

            } **/
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x:", poseEstimate.getX());
            telemetry.addData("y:", poseEstimate.getY());
            telemetry.addData("heading:", poseEstimate.getHeading());

            telemetry.update();
            drive.update();

        }
    }

}
