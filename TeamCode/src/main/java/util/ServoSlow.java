package util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoSlow {

    public ServoSlow(){}

    public void setPosition(Servo servo, double increment, double targetTime, double target, ElapsedTime time) {
        time.reset();
        double pos = servo.getPosition();

        if(time.milliseconds() <= targetTime && pos < target) {
            pos += increment;
            time.reset();
        }

        servo.setPosition(pos);
    }
}
