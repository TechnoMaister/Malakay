package util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {

    public Encoder(){}

    public void runTo(DcMotor motor, int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }
}
