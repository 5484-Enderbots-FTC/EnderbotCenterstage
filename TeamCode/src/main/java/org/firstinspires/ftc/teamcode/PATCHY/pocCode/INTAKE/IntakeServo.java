package org.firstinspires.ftc.teamcode.PATCHY.pocCode.INTAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//

@TeleOp(name = "Intake + Servo", group = "Linear Opmode")
@Disabled
public class IntakeServo extends LinearOpMode {

    public enum intakeState {
        intakeMiddle,
        intakeOut,
        intakeIn

    }

    public enum intakeSpin {
        intakeReverse,
        intakeForward,
        intakeStop
    }
    intakeState intakePos = intakeState.intakeOut;
    intakeSpin intakeMtr = intakeSpin.intakeStop;

    Servo intakeLeft;
    Servo intakeRight;
    Servo droneLauncher;

    DistanceSensor intakeOne;
    DistanceSensor intakeTwo;

    DcMotorEx mtrI;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime servoTime = new ElapsedTime();

    Character lastBtn = null;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeLeft = hardwareMap.get(Servo.class, "leftSvr");
        intakeRight = hardwareMap.get(Servo.class, "rightSvr");
        droneLauncher = hardwareMap.get(Servo.class, "svrDrone");

        mtrI = hardwareMap.get(DcMotorEx.class, "mtrI");
        mtrI.setZeroPowerBehavior(BRAKE);
        mtrI.setDirection(DcMotor.Direction.REVERSE);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.addData("left servo ", intakeLeft.getPosition());
        telemetry.addData("right servo ", intakeRight.getPosition());
        telemetry.addData("drone servo: ", droneLauncher.getPosition());
        telemetry.addData("servo time: ", servoTime.time());
        telemetry.update();

        //outside pos
        intakeRight.setPosition(0.83);
        intakeLeft.setPosition(0.34);

        while (opModeIsActive()) {
          /*  if (gamepad1.dpad_up) {
                mtrI.setDirection(DcMotor.Direction.REVERSE);
                mtrI.setPower(0.62);
            }
            if (gamepad1.dpad_down) {
                mtrI.setDirection(DcMotor.Direction.FORWARD);
                mtrI.setPower(0.62);
            }
            if (gamepad1.dpad_right) {
                mtrI.setPower(0);
            }

            */if (gamepad1.a) {
                intakeLeft.setPosition(intakeLeft.getPosition() - (gamepad1.left_stick_y * 0.001));
                intakeRight.setPosition(intakeRight.getPosition() - (gamepad1.right_stick_y * 0.001));
                droneLauncher.setPosition(intakeRight.getPosition() - (gamepad1.left_stick_x * 0.001));
            }
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
//open position
            if (gamepad1.y) {
                intakeLeft.setPosition(0);
                intakeRight.setPosition(1);
            }
  //middle
            if (gamepad1.x) {
                intakeRight.setPosition(0.93);
                intakeLeft.setPosition(.06);
            }

            telemetry.addData("left servo ", intakeLeft.getPosition());
            telemetry.addData("right servo ", intakeRight.getPosition());
            telemetry.addData("servo time: ", servoTime.time());
            telemetry.addData("last btn: ", lastBtn);
            telemetry.update();
        }
    }
}
