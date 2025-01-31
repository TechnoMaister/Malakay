package teleOp;

import static util.LiftVelocityPIDF.p;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;
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
import static util.RobotConstants.HANG;
import static util.RobotConstants.HIGH_CHAMBER;
import static util.RobotConstants.INTAKE;
import static util.RobotConstants.LOW_BASKET;
import static util.RobotConstants.UNEXT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    public Pose startPose = new Pose(0,0,0);
    public Hardware robot;
    public PIDFController controller;
    public Timer intakeTime, extSubT, submersibleTime, basket, timer;
    public Gamepad previousGamepad1, currentGamepad1;
    public boolean L1, CROSS, CIRCLE, TRIANGLE;
    public double pidf, clawWristPos, clawPos, clawRotPos, extPos;
    public int liftPos, liftTargetPos, clawRot;
    public static double

    intakeTimeV = 500,

    intakeExt = 1000,

    extSubTV = 500,

    submersibleTimeV = 500,

    submersibleTimeV2 = 1000,

    basketV = 900,

    timerV = 500;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class,LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        robot = new Hardware(hardwareMap);

        controller = new PIDFController(p, i, d, f);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        intakeTime = new Timer();
        extSubT = new Timer();
        submersibleTime = new Timer();
        basket = new Timer();
        timer = new Timer();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
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

        controller.setPIDF(p, i, d, f);

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

        if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper && !CIRCLE) {
            L1 = !L1;
            intakeTime.resetTimer();
            submersibleTime.resetTimer();
            extSubT.resetTimer();
            timer.resetTimer();
        }
        if(currentGamepad1.cross && !previousGamepad1.cross && !CIRCLE && !L1) CROSS = !CROSS;
        if(currentGamepad1.circle && !previousGamepad1.circle && CROSS && !L1) {
            basket.resetTimer();
            CIRCLE = !CIRCLE;
        }
        if(currentGamepad1.triangle && !previousGamepad1.triangle && CROSS && !L1 && !CIRCLE) TRIANGLE = !TRIANGLE;

        if(TRIANGLE) {
            if(L1) hang();
            else hangIdle();
        } else {
            if(CIRCLE) {
                scoreBasket();
            } else {
                if(basket.getElapsedTime() >= basketV) {
                    if(CROSS) {
                        if(L1) intakeSubmersible();
                        else idleSubmersible();
                    }
                    else {
                        if(L1) scoreSpecimen();
                        else intakeSpecimen();
                    }
                } else clawPos = CLAW_OPEN;
            }
        }
    }

    public void drive(Gamepad gamepad) {
        follower.setTeleOpMovementVectors(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
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
            clawWristPos = CLAW_UP_CHAMBER;
            if(intakeTime.getElapsedTime() >= intakeExt) extPos = EXT;
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

    public void hangIdle() {
        liftTargetPos = DOWN;
        extPos = UNEXT;
        clawWristPos = CLAW_MID;
        clawRot = 0;
        clawPos = CLAW_OPEN;
    }

    public void hang() {
        liftTargetPos = HANG;
        extPos = UNEXT;
        clawWristPos = CLAW_MID;
        clawRot = 0;
        clawPos = CLAW_OPEN;
    }
}
