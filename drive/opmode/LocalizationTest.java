package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ServosandSensors.servoConfig;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public DcMotor shooterMotor; DcMotor intakeMotor;
    servoConfig ServoConfig = new servoConfig();
    public Servo flingServo;
    //public Servo leftclampservo;
    //public Servo rightclampservo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterMotor = hardwareMap.get(DcMotor.class,"shooter_drive");
        flingServo = hardwareMap.get(Servo.class,"flingservo");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        //leftclampservo = hardwareMap.get(Servo.class,"leftclamp"); //Servo 1 = left
        //rightclampservo = hardwareMap.get(Servo.class,"rightclamp"); //servo 0 = right
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ServoConfig.initialize(this);
        waitForStart();
        //0.4 close for servo 1 \\ //0.6 open for servo 1
        //0.35 open for servo 0 \\ // 0.55 close for servo 0

        while (!isStopRequested()) {
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                    + VY_WEIGHT * Math.abs(baseVel.getY())
                    + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                    VX_WEIGHT * baseVel.getX(),
                    VY_WEIGHT * baseVel.getY(),
                    OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);
            double power = gamepad2.left_stick_y;
            power = Range.clip(power, -1,1);
            shooterMotor.setPower(power);
            if (gamepad2.b) {
                flingServo.setPosition(0.85);
            }
            if (gamepad2.a) {
                flingServo.setPosition(1);
            }

            if (gamepad1.x) {
                intakeMotor.setPower(0.5);
            }
            if (gamepad1.b) {
                intakeMotor.setPower(-0.5);
            }
            if (gamepad1.right_bumper) {
                intakeMotor.setPower(0);
            }
            if (gamepad1.a) {
                ServoConfig.conveyorDown();
            }

            if (gamepad1.y) {
                ServoConfig.conveyorReset();
            }
           /** if (gamepad2.x) {
                leftclampservo.setPosition(0.45);
                rightclampservo.setPosition(0.55);
            }
            if (gamepad2.y) {
                leftclampservo.setPosition(0.6);
                rightclampservo.setPosition(0.35);
            }*/

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("left",drive.leftRear.getCurrentPosition());
            telemetry.addData("right",drive.leftFront.getCurrentPosition());
            telemetry.addData("forward",drive.rightFront.getCurrentPosition());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
