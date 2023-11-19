package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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


@TeleOp(name = "testteleop", group = "Linear Opmode")
//@Disabled
public class CenterstageTeleop extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    //motors
    DcMotor mtrBL;
    DcMotor mtrBR;
    DcMotor mtrFL;
    DcMotor mtrFR;

    //servos
    Servo extend;
    Servo grab;

    Servo arm;
    Servo FRservo;
    Servo FLservo;
    Servo LED_strip;

    //sensors
    DistanceSensor sensorRange;

    //limit switches
    TouchSensor topLimit;
    TouchSensor lowLimit;
    DigitalChannel DistanceBreak;

    /**
     * -
     * <p>
     * CONSTANTS
     */
    //encoder stuff
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.9;
    static final double TURN_SPEED = 0.7;
    int PlacingMode = 1;
    boolean armOutButton = false;
    boolean gripCloseButton = false;
    boolean armOut = false;
    boolean gripClose = false;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // motors

        mtrBL = hardwareMap.get(DcMotor.class, "mtrBL");
        mtrBL.setZeroPowerBehavior(BRAKE);
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mtrBR = hardwareMap.get(DcMotor.class, "mtrBR");
        mtrBR.setZeroPowerBehavior(BRAKE);
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mtrFL = hardwareMap.get(DcMotor.class, "mtrFL");
        mtrFL.setZeroPowerBehavior(BRAKE);
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mtrFR = hardwareMap.get(DcMotor.class, "mtrFR");
        mtrFR.setZeroPowerBehavior(BRAKE);
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("heres the encoders", "Starting at %7d :%7d :%7d :%7d",
                mtrFL.getCurrentPosition(),
                mtrFR.getCurrentPosition(),
                mtrBL.getCurrentPosition(),
                mtrBR.getCurrentPosition());
        telemetry.update();


        //start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            /**

             GAMEPAD 1 CONTROLS
             **/

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * PlacingMode * (1 - (gamepad1.right_trigger * 0.5)),
                            gamepad1.left_stick_x * .8 * PlacingMode * (1 - (gamepad1.right_trigger * 0.5)),
                            gamepad1.right_stick_x * (1 - (gamepad1.right_trigger * 0.5))
                    )
            );


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());



            telemetry.addData("arm", armOut);
            telemetry.addData("armButton", armOutButton);

            telemetry.addData("grip", gripClose);
            telemetry.addData("gripButton", gripCloseButton);
            //grab.setPosition();


            telemetry.update();
            drive.update();







        }
    }

}
