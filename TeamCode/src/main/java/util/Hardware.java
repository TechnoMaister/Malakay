package util;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public Motor leftLift, rightLift;
    public Servo extend, clawWrist, clawRotation, claw;
    public MotorGroup lift;

    public Hardware(HardwareMap robot) {
        leftLift = new Motor(robot, "leftLift");
        rightLift = new Motor(robot, "rightLift");

        rightLift.setInverted(true);

        lift = new MotorGroup(leftLift, rightLift);

        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lift.setRunMode(Motor.RunMode.VelocityControl);

        lift.stopAndResetEncoder();

        extend = robot.get(Servo.class, "extend");
        clawWrist = robot.get(Servo.class, "clawWrist");
        clawRotation = robot.get(Servo.class, "clawRotation");
        claw = robot.get(Servo.class, "claw");
    }
}
