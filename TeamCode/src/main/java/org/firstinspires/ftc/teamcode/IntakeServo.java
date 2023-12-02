package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//

@TeleOp(name = "Intake Servo", group = "Linear Opmode")

public class IntakeServo extends LinearOpMode {

    Servo intakeLeft;
    Servo intakeRight;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime servoTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeLeft = hardwareMap.get(Servo.class, "leftSvr");
        intakeRight = hardwareMap.get(Servo.class, "rightSvr");

        waitForStart();


        intakeRight.setPosition(0.5);
        servoTime.reset();

        if (servoTime.time() > 1) {
            intakeLeft.setPosition(0.0);
        }
        while (opModeIsActive())
            if (gamepad1.a) {
                intakeLeft.setPosition(intakeLeft.getPosition() - (gamepad1.left_stick_y * 0.001));
                intakeRight.setPosition(intakeRight.getPosition() - (gamepad1.right_stick_y * 0.001));
                telemetry.addData("left servo ", intakeLeft.getPosition());
                telemetry.addData("right servo ", intakeRight.getPosition());
                telemetry.update();
            }
    }
}
