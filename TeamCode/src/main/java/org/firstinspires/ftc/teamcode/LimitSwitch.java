package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
@TeleOp(name = "LimitSwitch", group = "Linear OpMode")
//@Disabled
public class LimitSwitch {
    DcMotorEx mtrLift;
    TouchSensor bottomlimitSwitch;
    TouchSensor toplimitSwitch;

    public void setMotorSpeed(double speed) {

        mtrLift.setZeroPowerBehavior(BRAKE);

        if (speed > 0) {
            if (toplimitSwitch.isPressed()) {
                // We are going up and top limit is tripped so stop
                mtrLift.setPower(0);
            } else {
                // We are going up but top limit is not tripped so go at commanded speed
                mtrLift.setPower(speed);
            }
        } else {
            if (bottomlimitSwitch.isPressed()) {
                // We are going down and bottom limit is tripped so stop
                mtrLift.setPower(0);
            } else {
                // We are going down but bottom limit is not tripped so go at commanded speed
                mtrLift.setPower(speed);
            }
        }
    }
}