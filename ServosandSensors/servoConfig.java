package org.firstinspires.ftc.teamcode.ServosandSensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Superclass.Subsystem;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;

public class servoConfig extends Subsystem{
    Servo wobbleGoalLiftingServo;
    Servo wobbleGoalClampServo;
    Servo leftConveyorLiftingServo;
    Servo rightConveyorLiftingServo;
    @Override
    public void initialize(LinearOpMode opMode) {
        wobbleGoalLiftingServo = opMode.hardwareMap.get(Servo.class,"wobbleGoalLiftingServo");
        wobbleGoalClampServo = opMode.hardwareMap.get(Servo.class,"wobbleGoalClampServo");
        leftConveyorLiftingServo = opMode.hardwareMap.get(Servo.class,"leftConveyorLiftingServo");
        rightConveyorLiftingServo = opMode.hardwareMap.get(Servo.class,"rightConveyorLiftingServo");

    }
    public void autoWobbleHold() {
        wobbleGoalClampServo.setPosition(0.1);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        wobbleGoalLiftingServo.setPosition(1);
    }
    public void autoWobbleInitialize() {
        wobbleGoalLiftingServo.setPosition(1);
        wobbleGoalClampServo.setPosition(0.47);
    }
    public void autoWobbleUp() {

    }

    public void autoWobbleDown() {
        //Servo 0
        wobbleGoalLiftingServo.setPosition(0.3);
        wobbleGoalClampServo.setPosition(1);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void conveyorDown() {
        leftConveyorLiftingServo.setPosition(0.95);
        rightConveyorLiftingServo.setPosition(0.05);
    }
    public void conveyorUp() {
        leftConveyorLiftingServo.setPosition(0.2);
        rightConveyorLiftingServo.setPosition(0.8);
    }
    public void conveyorReset() {
        leftConveyorLiftingServo.setPosition(0.4);
        rightConveyorLiftingServo.setPosition(0.6);
    }

}
