package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Shooter", group="Linear Opmode")
public class shooterSystem extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor shooterMotor;
    public Servo flingServo;
    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotor.class,"fr_drive");
        flingServo = hardwareMap.get(Servo.class,"flingservo");
        //flingServo.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            double power = gamepad2.left_stick_y;
            power = Range.clip(power, -1,1);
            shooterMotor.setPower(power);
            if (gamepad2.b) {
                flingServo.setPosition(0.85);
            }
            if (gamepad2.a) {
                flingServo.setPosition(1);
            }
        }
    }
}
