package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(group = "drive")
public class ThreadDemo extends LinearOpMode
{

    DcMotor leftMotor, rightMotor,bottomLeftMotor,bottomRightMotor;
    float   leftY, rightY;
    // called when init button is  pressed.
    Servo testServo;
    @Override
    public void runOpMode() throws InterruptedException
    {

        Thread  driveThread = new DriveThread();
        testServo = hardwareMap.get(Servo.class,"testServo");
        leftMotor = hardwareMap.dcMotor.get("fl_drive");
        rightMotor = hardwareMap.dcMotor.get("fr_drive");
        bottomLeftMotor = hardwareMap.dcMotor.get("bl_drive");
        bottomRightMotor = hardwareMap.dcMotor.get("br_drive");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Mode", "waiting");
        telemetry.update();
        double x = 0.5;

        // wait for start button.

        waitForStart();

        // start the driving thread.

        driveThread.start();

        // continue with main thread.

        try
        {
            while (opModeIsActive())
            {
                testServo.setPosition(x);
                telemetry.addData("Mode", "running");
                telemetry.addData("Run Time", this.getRuntime());
                telemetry.update();
                sleep(500);
                x = -x;

                idle();
            }
        }
        catch(Exception e) {}

        // stop the driving thread.

        driveThread.interrupt();

    }

    private class SoundThread extends Thread {

    }

    private class DriveThread extends Thread
    {
        boolean g = false;
        public DriveThread()
        {
            this.setName("DriveThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {

            try
            {
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    leftY = gamepad1.left_stick_y * -1;
                    rightY = gamepad1.right_stick_y * -1;

                    leftMotor.setPower(Range.clip(leftY, -1.0, 1.0));
                    rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));
                    bottomLeftMotor.setPower(Range.clip(leftY, -1.0, 1.0));
                    bottomRightMotor.setPower(Range.clip(rightY, -1.0, 1.0));
                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            catch (Exception e) {}
        }
    }
}
