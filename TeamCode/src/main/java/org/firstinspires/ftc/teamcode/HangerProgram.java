package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
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
            mtrHang.setDirection(DcMotor.Direction.FORWARD);

            if (gamepad1.a && !limitSwitch.isPressed() && elapsedTime.time() > 60) {
                mtrHang.setPower(0.3);
                wait(5);
                mtrHang.setPower(0);

            } else if (gamepad1.b && limitSwitch.isPressed() && elapsedTime.time() > 60) {
                mtrHang.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrHang.setZeroPowerBehavior(BRAKE);
                wait(1);
                mtrHang.setPower(0.5);
                wait(3);
                mtrHang.setPower(0);
            }
        }
    }
}
