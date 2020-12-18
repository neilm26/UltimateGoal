package org.firstinspires.ftc.teamcode.drive.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ServosandSensors.Servos.servoConfig;
import org.firstinspires.ftc.teamcode.Superclass.Subsystem;

public class AutoConfig {
    public static servoConfig ServoConfig = new servoConfig();
    public  Subsystem[] subsystems = {ServoConfig};

    public void init(LinearOpMode opMode){
        for(int i=0; i < subsystems.length - 1; i++){
            subsystems[i].initialize(opMode);
        }
    }

    public AutoConfig() {

    }

}

