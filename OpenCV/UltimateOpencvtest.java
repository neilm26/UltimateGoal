package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="OpenCVRingTest", group="Opencv")
//@Disabled
public class UltimateOpencvtest extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    skystoneDetectorClass detector = new skystoneDetectorClass();

    int[] vals;


    @Override
    public void runOpMode() {
        detector.setOffset(0f / 8f, 0f / 8f);
        detector.camSetup(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        boolean JumpOut = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() && !JumpOut) {
            detector.updateVals();
            vals = detector.getVals();
            telemetry.addData("Ring Values",  " " + vals[0] + " " + vals[2]);
            telemetry.addData("Ring Position", detector.stone_loc(102));
            telemetry.update();
            sleep(100);

            if (detector.stone_loc(102) != 0) {
                JumpOut = true;
                //GLOBAL_SKYBLOCK_POS = detector.stone_loc(102);
                telemetry.addData("Jumped Out?", JumpOut);
                telemetry.update();
            }


        }
    }
}