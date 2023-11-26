package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//

@Disabled
@TeleOp(name = "IntakeServo", group = "Linear Opmode")

public class IntakeServo extends LinearOpMode {

    Servo IntakeLeft;
    Servo IntakeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        if (gamepad1.a) {
            IntakeLeft.setPosition(.6);
            IntakeRight.setPosition(.9);
        }  else if (!gamepad1.a){
            IntakeLeft.setPosition(0.2);
            IntakeRight.setPosition(0.2);
        }
    }
}
