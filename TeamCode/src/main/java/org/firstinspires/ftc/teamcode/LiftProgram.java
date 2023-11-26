package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "LiftProgram", group = "Linear Opmode")

public class LiftProgram extends LinearOpMode {

    DcMotor mtrHang;
    TouchSensor limitSwitch;

    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {



        mtrHang = hardwareMap.get(DcMotor.class, "mtrHang");
        mtrHang.setZeroPowerBehavior(BRAKE);
        mtrHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrHang.setDirection(DcMotor.Direction.FORWARD);

        if (gamepad1.a && !limitSwitch.isPressed() && elapsedTime.time() > 60) {
            mtrHang.setPower(0.5);
        }
    }
}
