package teleOp;

import static util.RobotConstants.CLAW_CLOSED;
import static util.RobotConstants.CLAW_DOWN;
import static util.RobotConstants.CLAW_MID;
import static util.RobotConstants.CLAW_OPEN;
import static util.RobotConstants.CLAW_ROT_45;
import static util.RobotConstants.CLAW_ROT_OR;
import static util.RobotConstants.CLAW_ROT_VR;
import static util.RobotConstants.CLAW_UP_BASKET;
import static util.RobotConstants.CLAW_UP_CHAMBER;
import static util.RobotConstants.CLAW_UP_SUB;
import static util.RobotConstants.DOWN;
import static util.RobotConstants.EXT;
import static util.RobotConstants.HIGH_CHAMBER;
import static util.RobotConstants.INTAKE;
import static util.RobotConstants.LIFT_D;
import static util.RobotConstants.LIFT_F;
import static util.RobotConstants.LIFT_I;
import static util.RobotConstants.LIFT_P;
import static util.RobotConstants.LOW_BASKET;
import static util.RobotConstants.UNEXT;
import static util.RobotConstants.basketV;
import static util.RobotConstants.extSubTV;
import static util.RobotConstants.intakeTimeV;
import static util.RobotConstants.submersibleTimeV;
import static util.RobotConstants.submersibleTimeV2;
import static util.RobotConstants.timerV;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import util.Hardware;

@Config
@TeleOp(group = "teleOp")
public class Drive extends OpMode {

    public Follower follower;
    public Pose startPose;
    public Hardware robot;
    public PIDFController controller;
    public Timer intakeTime, extSubT, submersibleTime, basket, timer;
    public Gamepad previousGamepad1, currentGamepad1;
    public boolean LB, A, B;
    public double pidf, clawWristPos, clawPos, clawRotPos, extPos;
    public int liftPos, liftTargetPos, clawRot;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        startPose = new Pose(0,0,0);

        robot = new Hardware(hardwareMap);

        controller = new PIDFController(LIFT_P, LIFT_I, LIFT_D, LIFT_F);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        intakeTime = new Timer();
        extSubT = new Timer();
        submersibleTime = new Timer();
        basket = new Timer();
        timer = new Timer();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        drive(gamepad1);

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        switch (clawRot) {
            case 0: {
                clawRotPos = CLAW_ROT_VR;
                break;
            }
            case 1: {
                clawRotPos = CLAW_ROT_45;
                break;
            }
            case 2: {
                clawRotPos = CLAW_ROT_OR;
                break;
            }
            case 3: {
                clawRot = 0;
                break;
            }
        }

        liftPos = robot.leftLift.getCurrentPosition();
        pidf = controller.calculate(liftPos, liftTargetPos);

        robot.lift.set(pidf);
        robot.clawWrist.setPosition(clawWristPos);
        robot.claw.setPosition(clawPos);
        robot.clawRotation.setPosition(clawRotPos);
        robot.extend.setPosition(extPos);

        if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper && !B) {
            LB = !LB;
            intakeTime.resetTimer();
            submersibleTime.resetTimer();
            extSubT.resetTimer();
            timer.resetTimer();
        }
        if(currentGamepad1.a && !previousGamepad1.a && !B && !LB) A = !A;
        if(currentGamepad1.b && !previousGamepad1.b && A && !LB) {
            basket.resetTimer();
            B = !B;
        }

        if(B) {
            scoreBasket();
        } else {
            if(basket.getElapsedTime() >= basketV) {
                if(A) {
                    if(LB) intakeSubmersible();
                    else idleSubmersible();
                }
                else {
                    if(LB) scoreSpecimen();
                    else intakeSpecimen();
                }
            } else clawPos = CLAW_OPEN;
        }
    }

    public void drive(Gamepad gamepad) {
        follower.setTeleOpMovementVectors(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true);
    }

    public void intakeSpecimen() {
        clawRot = 0;
        extPos = UNEXT;
        clawPos = CLAW_OPEN;
        if(timer.getElapsedTime() >= timerV) {
            liftTargetPos = INTAKE;
            clawWristPos = CLAW_MID;
        }
    }

    public void scoreSpecimen() {
        clawRot = 0;
        clawPos = CLAW_CLOSED;
        if(intakeTime.getElapsedTime() >= intakeTimeV) {
            liftTargetPos = HIGH_CHAMBER;
            extPos = EXT;
            clawWristPos = CLAW_UP_CHAMBER;
        } else extPos = UNEXT;
    }

    public void idleSubmersible() {
        liftTargetPos = DOWN;
        if(submersibleTime.getElapsedTime() >= submersibleTimeV)
            if(submersibleTime.getElapsedTime() >= submersibleTimeV2) {
                extPos = UNEXT;
                clawRot = 0;
                if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper) clawPos = CLAW_OPEN;
            }
            else clawWristPos = CLAW_UP_SUB;
        else clawPos = CLAW_CLOSED;
    }

    public void intakeSubmersible() {
        liftTargetPos = DOWN;
        if(extSubT.getElapsedTime() >= extSubTV) clawWristPos = CLAW_DOWN;
        else {
            extPos = EXT;
            clawPos = CLAW_OPEN;
        }
        if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper) clawRot++;
    }

    public void scoreBasket() {
        liftTargetPos = LOW_BASKET;
        clawWristPos = CLAW_UP_BASKET;
        clawRot = 2;
        clawPos = CLAW_CLOSED;
    }
}
