package org.firstinspires.ftc.teamcode.ServosandSensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Superclass.Subsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.EncoderTest;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class sensorConfig extends Subsystem {
    public Rev2mDistanceSensor dis_sensorleft;
    public Rev2mDistanceSensor dis_sensorright;
    public Rev2mDistanceSensor dis_sensorback;
    EncoderTest encoderTest = new EncoderTest();


    @Override
    public void initialize(LinearOpMode opMode) {
        dis_sensorleft = opMode.hardwareMap.get(Rev2mDistanceSensor.class,"dis_sensorleft");
        dis_sensorright = opMode.hardwareMap.get(Rev2mDistanceSensor.class,"dis_sensorright");
        dis_sensorback = opMode.hardwareMap.get(Rev2mDistanceSensor.class,"dis_sensorback");
        encoderTest.DriveInit(hardwareMap);
    }
    public void DisSensor(double leftSensorVal, double rightSensorVal, double backSensorVal, String Direction) {
        if (Direction=="strafeLeft") {
            if (dis_sensorleft.getDistance(DistanceUnit.INCH)-leftSensorVal<-5) {
                //strafe left
                while ((dis_sensorleft.getDistance(DistanceUnit.INCH)-leftSensorVal)<-5) {
                    telemetry.addData("dislefval",dis_sensorleft.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                    encoderTest.strafeLeft(0.3);
                }
            }
            else if (dis_sensorleft.getDistance(DistanceUnit.INCH)-leftSensorVal>5) {
                //strafe right
                while ((dis_sensorleft.getDistance(DistanceUnit.INCH)-leftSensorVal)>5) {
                    telemetry.addData("dislefval",dis_sensorleft.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                    encoderTest.strafeRight(0.3);
                }
            }
        }
        if (Direction=="strafeRight") {
            if (dis_sensorright.getDistance(DistanceUnit.INCH)-leftSensorVal<-5)
                while (Math.abs(dis_sensorright.getDistance(DistanceUnit.INCH)-rightSensorVal)>5) {

            }
        }
        if (Direction=="verticalMovement") {
            while (Math.abs(dis_sensorback.getDistance(DistanceUnit.INCH)-backSensorVal)>5) {

            }
        }

    }


    public List readDistanceInch(Rev2mDistanceSensor...sensors){
        List<Double> myList = new ArrayList<Double>();
        for (Rev2mDistanceSensor sensor:sensors){
            myList.add(sensor.getDistance(DistanceUnit.INCH)) ;
        }
        return myList;
    }
    public List readDistanceCM(Rev2mDistanceSensor...sensors){
        List<Double> myList = new ArrayList<Double>();
        for (Rev2mDistanceSensor sensor:sensors){
            myList.add(sensor.getDistance(DistanceUnit.CM)) ;
        }
        return myList;
    }
    public boolean checkReading(Rev2mDistanceSensor sensor){
        boolean result = true;
        for(int i =0; i < 5; i++){
            if(sensor.getDistance(DistanceUnit.INCH) <100){
                result = true;
                break;
            } else {
                result = false;
            }
        }
        return result;
    }
}
