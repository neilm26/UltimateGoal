import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Superclass.Subsystem;
import java.util.ArrayList;
import java.util.List;

public class servoConfig extends Subsystem{

Servo clamp;
@Override
    public void initialize(LinearOpMode opMode) {
        clamp = opMode.hardwareMap.get(Servo.class,"clamp");
        clamp.setPosition(0.47);
   }
   public void open(){
   clamp.setPosition(1);
   }
   public void close(){
   clamp.setPosition(0.1);
   }
   
 }
