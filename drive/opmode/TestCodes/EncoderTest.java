package org.firstinspires.ftc.teamcode.drive.opmode.TestCodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

@Autonomous(group = "drive")

public class EncoderTest extends LinearOpMode {
    public static double TICKS_PER_REV = 1600;
    public static double WHEEL_RADIUS = 0.29; // in
    public static double GEAR_RATIO = 1.5;

    static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * GEAR_RATIO) /
            (WHEEL_RADIUS * 2 * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public double leftFrontStartPos;
    public double rightFrontStartPos;
    public double leftRearStartPos;
    public double avgCurrentPos;
    public double avgTargetPos;
    public boolean finished;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveInit(hardwareMap);
        //setEncoderPosition(15,15);
        //telemetry.addData("left target",setEncoderPosition(15,15));
        //telemetry.update();
        leftFrontStartPos = Math.abs(leftFront.getCurrentPosition());
        rightFrontStartPos = Math.abs(rightFront.getCurrentPosition());
        leftRearStartPos = Math.abs(leftRear.getCurrentPosition());
        setTargetPositionStrafe(15);
       // setTargetPosition(25,25);
        waitForStart();
        while (avgCurrentPos < avgTargetPos && opModeIsActive()) {
            avgCurrentPos  = (Math.abs(leftRear.getCurrentPosition()));
            telemetry.addData("averagecurrent",avgCurrentPos);
            telemetry.addData("avgtarget",avgTargetPos);
            telemetry.addData("Difference",avgCurrentPos-avgTargetPos);
            telemetry.update();
            //encoderStrafe(-0.2);
        }
        stopAll();
    }

    public void DriveInit(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "fl_drive");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl_drive");
        rightRear = hardwareMap.get(DcMotorEx.class, "br_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr_drive");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void setTargetPosition(double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = (int) ((leftInches*COUNTS_PER_INCH));
        newRightTarget = (int) ((rightInches*COUNTS_PER_INCH));
        avgTargetPos = (leftFrontStartPos+rightFrontStartPos)/2+(newLeftTarget+newRightTarget)/2;
    }
    public void setTargetPositionStrafe(double horizontalInches) {
        int newHorizontalTarget;

        newHorizontalTarget = (int) (horizontalInches*COUNTS_PER_INCH);
        avgTargetPos = (leftRearStartPos+newHorizontalTarget);
    }
    public void encoderDriveStraight(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);
    }
    public void strafeLeft(double speed) {
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(-speed);
    }
    public void strafeRight(double speed) {
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftRear.setPower(-speed);
        rightRear.setPower(speed);
    }


    public void stopAll() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

}
