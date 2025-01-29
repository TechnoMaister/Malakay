package teleOp;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
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

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import util.Hardware;

@Config
@TeleOp(group = "teleOp")
public class Drive extends OpMode {

    public Hardware robot;
    public PIDFController controller;
    public ElapsedTime intakeTime, extSubT, submersibleTime, basket, timer;
    public Gamepad previousGamepad1, currentGamepad1;
    public boolean LB, A, B;
    double pidf, clawWristPos, clawPos, clawRotPos, extPos;
    int liftPos, liftTargetPos, clawRot;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        intakeTime = new ElapsedTime();
        extSubT = new ElapsedTime();
        submersibleTime = new ElapsedTime();
        basket = new ElapsedTime();
        timer = new ElapsedTime();

        controller = new PIDFController(LIFT_P, LIFT_I, LIFT_D, LIFT_F);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);
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
            intakeTime.reset();
            submersibleTime.reset();
            extSubT.reset();
            timer.reset();
        }
        if(currentGamepad1.a && !previousGamepad1.a && !B && !LB) A = !A;
        if(currentGamepad1.b && !previousGamepad1.b && A && !LB) {
            basket.reset();
            B = !B;
        }

        if(B) {
            scoreBasket();
        } else {
            if(basket.milliseconds() >= 900) {
                if(A) {
                    if(LB) intakeSubmersible();
                    else idleSubmersible();
                }
                else {
                    if(LB) scoreSpecimen2();
                    else intakeSpecimen2();
                }
            } else clawPos = CLAW_OPEN;
        }
    }

    public void drive(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x; // this is strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    public void intakeSpecimen() {
        clawRot = 0;
        extPos = UNEXT;
        if(timer.milliseconds() >= 500) {
            clawWristPos = CLAW_MID;
            liftTargetPos = INTAKE;
            if(timer.milliseconds() >= 750) clawPos = CLAW_OPEN;
        } else {
            clawWristPos = CLAW_DOWN;
            clawPos = CLAW_CLOSED;
        }
    }

    public void scoreSpecimen() {
        clawRot = 0;
        clawPos = CLAW_CLOSED;
        extPos = UNEXT;
        if(intakeTime.milliseconds() >= 500) liftTargetPos = HIGH_CHAMBER;
    }

    public void intakeSpecimen2() {
        clawRot = 0;
        extPos = UNEXT;
        clawPos = CLAW_OPEN;
        if(timer.milliseconds() >= 200) {
            liftTargetPos = INTAKE;
            clawWristPos = CLAW_MID;
        }
    }

    public void scoreSpecimen2() {
        clawRot = 0;
        clawPos = CLAW_CLOSED;
        if(intakeTime.milliseconds() >= 500) {
            liftTargetPos = HIGH_CHAMBER;
            extPos = EXT;
            clawWristPos = CLAW_UP_CHAMBER;
        } else extPos = UNEXT;
    }

    public void idleSubmersible() {
        liftTargetPos = DOWN;
        if(submersibleTime.milliseconds() >= 500)
            if(submersibleTime.milliseconds() >= 1000) {
                extPos = UNEXT;
                clawRot = 0;
                if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper) clawPos = CLAW_OPEN;
            }
            else clawWristPos = CLAW_UP_SUB;
        else clawPos = CLAW_CLOSED;
    }

    public void intakeSubmersible() {
        liftTargetPos = DOWN;
        if(extSubT.milliseconds() >= 500) clawWristPos = CLAW_DOWN;
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
