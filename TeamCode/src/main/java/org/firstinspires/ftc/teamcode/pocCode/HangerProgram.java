package org.firstinspires.ftc.teamcode.pocCode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Hanger Program", group = "Linear Opmode")

public class HangerProgram extends LinearOpMode {

    DcMotor mtrHang;
    TouchSensor limitSwitch;

    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeIsActive()) {

            mtrHang = hardwareMap.get(DcMotor.class, "mtrHang");
            mtrHang.setZeroPowerBehavior(BRAKE);
            mtrHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtrHang.setDirection(DcMotor.Direction.REVERSE);

            if (gamepad2.a) { //&& !limitSwitch.isPressed() && runtime.time() > 60) { <- comment back in when ready
                mtrHang.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrHang.setPower(0.3);

            } else if (gamepad2.b) {//&& limitSwitch.isPressed() && runtime.time() > 60) { <- comment back in when ready
                mtrHang.setDirection(DcMotor.Direction.FORWARD);
                mtrHang.setZeroPowerBehavior(BRAKE);
                mtrHang.setPower(0.5);
            }

        }
    }
}
