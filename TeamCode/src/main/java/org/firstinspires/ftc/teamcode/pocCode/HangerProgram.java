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

            while (gamepad1.a /*&& elapsedTime.time() > 60)*/) {
                mtrHang.setDirection(DcMotorSimple.Direction.REVERSE);
                mtrHang.setPower(0.3);

                if (!gamepad1.a) {
                    mtrHang.setPower(0);
                    break;
                }

            while  (gamepad1.b) {
                mtrHang.setDirection(DcMotor.Direction.FORWARD);
                mtrHang.setPower(0.5 * (1 - (gamepad1.right_trigger * 0.6)));

                if (!gamepad1.b) {
                    mtrHang.setPower(0);
                    break;
                }

            }

            }

        }
    }
}
