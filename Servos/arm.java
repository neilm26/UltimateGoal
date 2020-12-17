import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Superclass.Subsystem;
import java.util.ArrayList;
import java.util.List;

public class servoConfig extends Subsystem{

Servo arm;
@Override
    public void initialize(LinearOpMode opMode) {
        arm = opMode.hardwareMap.get(Servo.class,"arm");
        arm.setPosition(1);
   }
   public void up(){
   //arm.setPosition(1);
   //no value avaliable 
   }
   public void down(){
   arm.setPosition(0.3);
   }
   
 }
