package org.firstinspires.ftc.teamcode.PATCHY.pocCode.MECHANISMS;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadrunnerUtilStuff.drive.SampleMecanumDrive;
//

@TeleOp(name = "dronetest", group = "Linear Opmode")
//@Disabled
@Disabled
public class dronetest extends LinearOpMode {

    Servo droneLauncher;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        droneLauncher = hardwareMap.get(Servo.class, "svrDrone");

        droneLauncher.setPosition(1.0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                droneLauncher.setPosition(.689);
            }
            droneLauncher.setPosition(droneLauncher.getPosition() - (gamepad1.left_stick_y * 0.001));

            telemetry.addData("servo pos", droneLauncher.getPosition());
            telemetry.update();

        }
    }

    //ElapsedTime servoTime = new ElapsedTime();

    }

