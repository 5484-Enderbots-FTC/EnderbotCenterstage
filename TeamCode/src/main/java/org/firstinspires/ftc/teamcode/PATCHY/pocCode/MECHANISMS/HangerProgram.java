package org.firstinspires.ftc.teamcode.PATCHY.pocCode.MECHANISMS;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Hanger Program", group = "Linear Opmode")
@Disabled
public class HangerProgram extends LinearOpMode {

    DcMotor mtrHang;
    TouchSensor limitSwitch;

    Servo svrHang;

    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {

        mtrHang = hardwareMap.get(DcMotor.class, "mtrHang");
        mtrHang.setZeroPowerBehavior(BRAKE);
        mtrHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrHang.setDirection(DcMotor.Direction.REVERSE);

        svrHang = hardwareMap.get(Servo.class, "svrHang");
        svrHang.setPosition(0.53);

        waitForStart();

        while (opModeIsActive()) {
            while (!isStopRequested()) {
//down
                if (gamepad1.a /*&& elapsedTime.time() > 60)*/) {
                    mtrHang.setDirection(DcMotorSimple.Direction.REVERSE);
                    mtrHang.setPower(0.9);
                }
//up
                if (gamepad1.b) {
                    mtrHang.setDirection(DcMotor.Direction.FORWARD);
                    mtrHang.setPower(0.7);

                }

               if (!gamepad1.a && !gamepad1.b) {
                    mtrHang.setPower(0);
                }

                if (gamepad1.x) {
                    svrHang.setPosition(.9);
                }

                telemetry.addData("Servo Pos: ", svrHang.getPosition());
                telemetry.update();
            }
        }
    }
}
