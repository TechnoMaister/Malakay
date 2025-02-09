package util;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

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

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, extend;
    public List<DcMotorEx> motors, lift;
    public Servo clawWrist, clawRotation, claw;

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
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        leftLift = robot.get(DcMotorEx.class, "leftLift");
        rightLift = robot.get(DcMotorEx.class, "rightLift");

        rightLift.setDirection(DcMotorEx.Direction.REVERSE);
        leftLift.setDirection(DcMotorEx.Direction.FORWARD);

        lift = Arrays.asList(leftLift, rightLift);

        for (DcMotorEx motor : lift) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        extend = robot.get(DcMotorEx.class, "extend");

        extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        clawWrist = robot.get(Servo.class, "clawWrist");
        clawRotation = robot.get(Servo.class, "clawRotation");
        claw = robot.get(Servo.class, "claw");
    }
}
