package teleOp;

import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.p;
import static util.RobotConstants.CLAW_CLOSED;
import static util.RobotConstants.CLAW_DOWN;
import static util.RobotConstants.CLAW_MID;
import static util.RobotConstants.CLAW_OPEN;
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
import static util.RobotConstants.MEXT;
import static util.RobotConstants.UNEXT;

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
import util.Encoder;
import util.Hardware;

@Config
@TeleOp(group = "teleOp")
public class Drive2 extends OpMode {

    public Follower follower;
    public Pose startPose = new Pose(0,0,0);
    public Hardware robot;
    public Encoder encoder;
    public PIDFController liftController;
    public Timer intakeTime, extSubT, submersibleTime, basket, timer;
    public Gamepad previousGamepad1, currentGamepad1;
    public boolean L1, CROSS, CIRCLE, TRIANGLE;
<<<<<<< Updated upstream
    public double pidf;
    public int liftPos, liftTargetPos, extTargetPos, clawRot;
=======
    public static double pidf;
    public static int liftPos, liftTargetPos, extTargetPos, clawRot;
>>>>>>> Stashed changes

    public static double clawPos = CLAW_OPEN;
    public static double clawWristPos = CLAW_MID;
    public static double clawRotPos = CLAW_ROT_VR;

<<<<<<< Updated upstream
    public static double
            intakeTimeV = 500,
            intakeExt = 1000,
            extSubTV = 500,
            submersibleTimeV = 500,
            submersibleTimeV2 = 1000,
            basketV = 900,
            timerV = 500;
=======
    public static int
        liftSus = 2500,
        liftJos = 0;

>>>>>>> Stashed changes

    @Override
    public void init() {
        Constants.setConstants(FConstants.class,LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

<<<<<<< Updated upstream
=======
        liftTargetPos = liftJos;
>>>>>>> Stashed changes
        robot = new Hardware(hardwareMap);
        encoder = new Encoder();

        liftController = new PIDFController(p, i, d, f);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
<<<<<<< Updated upstream
=======
        extTargetPos = 200;
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
        if(gamepad1.dpad_down) clawPos = CLAW_CLOSED;
        else if(gamepad1.dpad_up) clawPos = CLAW_OPEN;

        if(gamepad1.dpad_left) clawWristPos = CLAW_DOWN;
        else if(gamepad1.dpad_right) clawWristPos = CLAW_UP_BASKET;

        if(gamepad1.left_bumper) clawRotPos = CLAW_ROT_VR;
        else if(gamepad1.right_bumper) clawRotPos = CLAW_ROT_OR;
=======
        drive(gamepad1);
>>>>>>> Stashed changes

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

<<<<<<< Updated upstream
        // robot.lift.set(pidf);
=======
        if(gamepad1.dpad_down) clawPos = CLAW_CLOSED;
        else if(gamepad1.dpad_up) clawPos = CLAW_OPEN;

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) clawRotPos = CLAW_DOWN;

        if(currentGamepad1.dpad_left) {
            clawWristPos = CLAW_DOWN;
        } else if(currentGamepad1.dpad_right) {
            clawWristPos = CLAW_UP_BASKET;
        };

        if(currentGamepad1.left_trigger != 0) clawRotPos = CLAW_ROT_VR;
        else if(currentGamepad1.right_trigger != 0) clawRotPos = CLAW_ROT_OR;

        if (currentGamepad1.triangle) {
            liftTargetPos = liftSus;
        } else if (currentGamepad1.cross) {
            liftTargetPos = liftJos;
        }


        liftController.setPIDF(p, i, d, f);

        liftPos = robot.leftLift.getCurrentPosition();
        pidf = liftController.calculate(liftPos, liftTargetPos);

        robot.lift.set(pidf);
>>>>>>> Stashed changes

        encoder.runTo(robot.extend, extTargetPos);
        robot.clawWrist.setPosition(clawWristPos);
        robot.claw.setPosition(clawPos);
        robot.clawRotation.setPosition(clawRotPos);

<<<<<<< Updated upstream
        //Ma fut pe el dashboard si pe el robot <3 Doamne Ajuta.
=======
>>>>>>> Stashed changes
        telemetry.addData("Claw Position Set", clawPos);
        telemetry.addData("Claw Actual Position", robot.claw.getPosition());
        telemetry.addData("ClawWrist Position Set", clawWristPos);
        telemetry.addData("ClawWrist Actual Position", robot.clawWrist.getPosition());
        telemetry.addData("ClawRot Position Set", clawRotPos);
        telemetry.addData("ClawRot Actual Position", robot.clawRotation.getPosition());
        telemetry.addData("L1", L1);
        telemetry.addData("CROSS", CROSS);
        telemetry.addData("TRIANGLE", TRIANGLE);
        telemetry.addData("CIRCLE", CIRCLE);
        telemetry.addData("BASKET TIME", basket.getElapsedTime());
        telemetry.addData("Timer", timer.getElapsedTime());
        telemetry.addData("Current Function", L1 ? "scoreSpecimen" : "intakeSpecimen");
        telemetry.update();
    }

    public void drive(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }

<<<<<<< Updated upstream
    public void intakeSpecimen() {
        clawRot = 0;
        extTargetPos = UNEXT;
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
            if(intakeTime.getElapsedTime() >= intakeExt) extTargetPos = MEXT;
        } else extTargetPos = UNEXT;
    }

    public void idleSubmersible() {
        liftTargetPos = DOWN;
        if(submersibleTime.getElapsedTime() >= submersibleTimeV)
            if(submersibleTime.getElapsedTime() >= submersibleTimeV2) {
                extTargetPos = UNEXT;
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
            extTargetPos = EXT;
            clawPos = CLAW_OPEN;
        }
        if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper) clawRot++;
    }

    public void scoreBasket() {
        liftTargetPos = LOW_BASKET;
        extTargetPos = MEXT;
        clawWristPos = CLAW_UP_BASKET;
        clawRot = 2;
        clawPos = CLAW_CLOSED;
    }

    public void hangIdle() {
        liftTargetPos = 0;
        extTargetPos = UNEXT;
        clawWristPos = CLAW_MID;
        clawRot = 0;
        clawPos = CLAW_OPEN;
    }

    public void hang() {
        liftTargetPos = HANG;
        extTargetPos = UNEXT;
        clawWristPos = CLAW_MID;
        clawRot = 0;
        clawPos = CLAW_OPEN;
    }
=======

>>>>>>> Stashed changes
}