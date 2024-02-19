package org.firstinspires.ftc.teamcode.PATCHY.pocCode.INTAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//

@TeleOp(name = "INTAKE SERVO ONLY", group = "Linear Opmode")
@Disabled
public class servoOnly extends LinearOpMode {

    Servo intakeLeft;
    Servo intakeRight;

    DcMotorEx mtrI;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime servoTime = new ElapsedTime();

    Character lastBtn = null;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeLeft = hardwareMap.get(Servo.class, "leftSvr");
        intakeRight = hardwareMap.get(Servo.class, "rightSvr");


        intakeRight.setPosition(1.0);
        intakeLeft.setPosition(0.0272);

        telemetry.addData("left servo ", intakeLeft.getPosition());
        telemetry.addData("right servo ", intakeRight.getPosition());
        telemetry.addData("servo time: ", servoTime.time());
        telemetry.update();

        waitForStart();

        //outside pos

        while (opModeIsActive()) {

            intakeLeft.setPosition(intakeLeft.getPosition() - (gamepad1.left_stick_y * 0.001));
            intakeRight.setPosition(intakeRight.getPosition() - (gamepad1.right_stick_y * 0.001));

            //middle pos

            //after x seconds have passed, what button was last pressed? then set position.

            telemetry.addData("left servo ", intakeLeft.getPosition());
            telemetry.addData("right servo ", intakeRight.getPosition());
            telemetry.addData("servo time: ", servoTime.time());
            telemetry.addData("last btn: ", lastBtn);
            telemetry.addData("gamepad right stick: ", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}