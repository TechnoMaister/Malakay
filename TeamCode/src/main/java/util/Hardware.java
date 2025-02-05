package util;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Hardware {

    public Motor leftLift, rightLift;
    public DcMotor extend;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public List<DcMotorEx> motors;
    public Servo clawWrist, clawRotation, claw;
    public MotorGroup lift;

    public Hardware(HardwareMap robot) {
        Constants.setConstants(FConstants.class, LConstants.class);

        leftFront = robot.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = robot.get(DcMotorEx.class, leftRearMotorName);
        rightRear = robot.get(DcMotorEx.class, rightRearMotorName);
        rightFront = robot.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftLift = new Motor(robot, "leftLift");
        rightLift = new Motor(robot, "rightLift");

        rightLift.setInverted(true);

        lift = new MotorGroup(leftLift, rightLift);

        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lift.setRunMode(Motor.RunMode.VelocityControl);

        lift.stopAndResetEncoder();

        extend = robot.get(DcMotor.class, "extend");

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawWrist = robot.get(Servo.class, "clawWrist");
        clawRotation = robot.get(Servo.class, "clawRotation");
        claw = robot.get(Servo.class, "claw");
    }
}
