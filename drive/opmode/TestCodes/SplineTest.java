package org.firstinspires.ftc.teamcode.drive.opmode.TestCodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.StoneOrientationExample;
import org.firstinspires.ftc.teamcode.ServosandSensors.Servos.servoConfig;
import org.firstinspires.ftc.teamcode.drive.Robot.AutoConfig;
import org.firstinspires.ftc.teamcode.drive.Robot.SampleMecanumDrive;

import java.util.Random;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    //Servo testServo;
    StoneOrientationExample orientationExample = new StoneOrientationExample();
    AutoConfig robot = new AutoConfig();
    private ElapsedTime runtime = new ElapsedTime();
    servoConfig ServoConfig = new servoConfig();
    //sensorConfig SensorConfig = new sensorConfig();
    double xPos;
    double yPos;

    boolean locationA;
    boolean locationB;
    boolean locationC;
    double x = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Thread  driveThread = new SplineTest.DriveThread();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        orientationExample.Initialization(hardwareMap);
        ServoConfig.initialize(this);

        //SensorConfig.initialize(this);
        robot.init(this);
        //ServoConfig.autoWobbleHold();


        runtime.reset();

        waitForStart();

        driveThread.start();

        if (isStopRequested()) return;
        Random rand = new Random();
        int randomNum = rand.nextInt((3 - 1) + 1) + 1;

        //orientationExample.GetStartingState();
        // nextInt as provided by Random is exclusive of the top value so you need to add 1
        if (randomNum==1) {locationA=true;} else if (randomNum==2) {locationB=true;} else {locationC=true;};
        xPos = (randomNum==1) ? -10: (randomNum==2) ? -40: (randomNum==3) ? 20 : 0;
        yPos = (randomNum==1) ? -50: (randomNum==2) ? -55: (randomNum==3) ? -30 : 0;
        //orientationExample.disable(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-60,-35));
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-60,-35))
                .splineTo(new Vector2d(-10,-15),0) //(B)
                .splineTo(new Vector2d(xPos,yPos),Math.toRadians(-40))
                .addTemporalMarker(5,() -> {

                    String header =
                            "**********************************\n" +
                                    "VOLTAGE MONITORS EXAMPLE          \n" +
                                    "**********************************\n";
                    telemetry.addLine(header);
                    telemetry.update();
                })
                .build();
        drive.followTrajectory(trajectory);
        sleep(100);
        drive.setPoseEstimate(new Pose2d(xPos,yPos,Math.toRadians(-40)));
        if (locationC) {
            //ServoConfig.autoWobbleDown();
            sleep(2000);
            // ServoConfig.autoWobbleInitialize();
            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(xPos,yPos,Math.toRadians(-40)),true)
                    .splineTo(new Vector2d(10,-15),0)
                    .build();
            drive.followTrajectory(trajectory2);
            driveThread.interrupt();
        }
        if (locationA) {
            //ServoConfig.autoWobbleDown();
            sleep(2000);
            //ServoConfig.autoWobbleInitialize();
            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(xPos,yPos,Math.toRadians(-40)),true)
                    .splineTo(new Vector2d(10,-27),0)
                    .build();
            drive.followTrajectory(trajectory2);
            driveThread.interrupt();
        }
        if (locationB) {
            //ServoConfig.autoWobbleDown();
            sleep(2000);
            //ServoConfig.autoWobbleInitialize();
            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(xPos,yPos,Math.toRadians(-40)),true)
                    //.splineTo(new Vector2d(30,-45),Math.toRadians(40))
                    .splineTo(new Vector2d(6,-10),0)
                    .build();
            drive.followTrajectory(trajectory2);
            driveThread.interrupt();

        }
        sleep(3000);
        ServoConfig.autoWobbleDown();

        drive.setPoseEstimate(new Pose2d(10,-15,Math.toRadians(180)));
        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d(10,-15,Math.toRadians(180)))
                .splineTo(new Vector2d(-34,-34),Math.toRadians(-135))
                .build();
        drive.followTrajectory(trajectory3);
        driveThread.interrupt();
        ServoConfig.autoWobbleHold();
        drive.setPoseEstimate(new Pose2d(-34,-34,Math.toRadians(-135)));

        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d(-34,-34,Math.toRadians(-135)))
                //.splineTo(new Vector2d(40,-35),0) (A)
                .splineTo(new Vector2d(-10,-15),0) //(B)
                .splineTo(new Vector2d(xPos,yPos),Math.toRadians(-40))
                //.splineTo(new Vector2d(10,-60),0)
                .addTemporalMarker(5,() -> {

                    String header =
                            "**********************************\n" +
                                    "VOLTAGE MONITORS EXAMPLE          \n" +
                                    "**********************************\n";
                    telemetry.addLine(header);
                    telemetry.update();
                })
                .build();
        drive.followTrajectory(trajectory4);
        sleep(100);
        if (locationC) {
            //ServoConfig.autoWobbleDown();
            sleep(2000);
            // ServoConfig.autoWobbleInitialize();
            drive.setPoseEstimate(new Pose2d(20,-30,Math.toRadians(-40)));
            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(20,-30,Math.toRadians(-40)),true)
                    .splineTo(new Vector2d(10,-15),0)
                    .build();
            drive.followTrajectory(trajectory2);
            driveThread.interrupt();
        }
        if (locationA) {
            //ServoConfig.autoWobbleDown();
            sleep(2000);
            //ServoConfig.autoWobbleInitialize();
            drive.setPoseEstimate(new Pose2d(-10,-55,Math.toRadians(-40)));
            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(-10,-55,Math.toRadians(-40)),true)
                    .splineTo(new Vector2d(10,-27),0)
                    .build();
            drive.followTrajectory(trajectory2);
            driveThread.interrupt();

        }
        if (locationB) {
            //ServoConfig.autoWobbleDown();
            sleep(2000);
            //ServoConfig.autoWobbleInitialize();
            drive.setPoseEstimate(new Pose2d(40,-55,Math.toRadians(-40)));
            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d(40,-55,Math.toRadians(-40)),true)
                    //.splineTo(new Vector2d(30,-45),Math.toRadians(40))
                    .splineTo(new Vector2d(6,-10),0)
                    .build();
            drive.followTrajectory(trajectory2);
            driveThread.interrupt();
        }

    }

    private class DriveThread extends Thread {

        public DriveThread() {
            this.setName("DriveThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {

            try {
                while (!isInterrupted()) {
                    //testServo.setPosition(x);
                    sleep(500);
                    x = -x;

                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            catch (Exception e) {
            }
        }
    }
}
