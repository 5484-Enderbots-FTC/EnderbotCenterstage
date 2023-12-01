package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//

@TeleOp(name = "IntakeServo", group = "Linear Opmode")

public class IntakeServo extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Servo intakeLeft;
   // Servo IntakeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeLeft = hardwareMap.get(Servo.class, "leftSvr");

        if (gamepad1.a) {
            intakeLeft.setPosition(0.5 + (gamepad1.left_stick_y * 0.1));
          //  IntakeRight.setPosition(.9);
        }  else if (!gamepad1.a){
            intakeLeft.setPosition(0.5 - (gamepad1.left_stick_y * 0.1));
          //  IntakeRight.setPosition(0.2);
        }

    }
}
