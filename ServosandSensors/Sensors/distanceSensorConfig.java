package org.firstinspires.ftc.teamcode.ServosandSensors.Sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Superclass.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class distanceSensorConfig extends Subsystem {
    Rev2mDistanceSensor dis_sensorleft;
    Rev2mDistanceSensor dis_sensorright;
    Rev2mDistanceSensor dis_sensorrback;

    @Override
    public void initialize(LinearOpMode opMode) {
        dis_sensorleft = opMode.hardwareMap.get(Rev2mDistanceSensor.class,"dis_sensorleft");
        dis_sensorright = opMode.hardwareMap.get(Rev2mDistanceSensor.class,"dis_sensorright");
        dis_sensorrback = opMode.hardwareMap.get(Rev2mDistanceSensor.class,"dis_sensorback");
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
