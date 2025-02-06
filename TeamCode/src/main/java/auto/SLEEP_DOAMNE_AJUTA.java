package auto;

import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.p;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import util.Encoder;
import util.Hardware;

@Autonomous
@Config
public class SLEEP_DOAMNE_AJUTA extends LinearOpMode {

    public Hardware robot;
    public Encoder encoder;
    public PIDFController liftController;
    public double pidf;
    public int liftPos, liftTargetPos;
    public static long time45 = 400, time90 = 850, time180 = 1600, time180offset = 100, chamber = 700, intermediate1 = 1100, intermediate2 = 500,
            rightSample = 500, push = 500, timeout = 300, scoreto = 500;
    public static double velocity = 0.75, moves = 6;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);
        encoder = new Encoder();
        liftController = new PIDFController(p, i, d, f);

        waitForStart();

        while (opModeIsActive()) {
            y(chamber, false);
            timeout(timeout);
            turn(90, 0, false);
            timeout(timeout);
            y(intermediate1, false);
            timeout(timeout);
            turn(90, 0, true);
            timeout(timeout);
            y(intermediate2, false);
            timeout(timeout);
            turn(90, 0, false);
            end();
        }
    }

    public void y(long time, Boolean reverse) {
        if(reverse) {
            robot.leftFront.setVelocity(-velocity*2781.1);
            robot.rightFront.setVelocity(-velocity*2781.1);
            robot.leftRear.setVelocity(-velocity*2781.1);
            robot.rightRear.setVelocity(-velocity*2781.1);
            sleep(time);
        } else {
            robot.leftFront.setVelocity(velocity*2781.1);
            robot.rightFront.setVelocity(velocity*2781.1);
            robot.leftRear.setVelocity(velocity*2781.1);
            robot.rightRear.setVelocity(velocity*2781.1);
            sleep(time);
        }
    }

    public void x(long time, Boolean reverse) {
        if(reverse) {
            robot.leftFront.setVelocity(-velocity*2781.1);
            robot.rightFront.setVelocity(velocity*2781.1);
            robot.leftRear.setVelocity(velocity*2781.1);
            robot.rightRear.setVelocity(-velocity*2781.1);
            sleep(time);
        } else {
            robot.leftFront.setVelocity(velocity*2781.1);
            robot.rightFront.setVelocity(-velocity*2781.1);
            robot.leftRear.setVelocity(-velocity*2781.1);
            robot.rightRear.setVelocity(velocity*2781.1);
            sleep(time);
        }
    }

    public void turn(double angle, long offset, Boolean left) {
        if(angle == 90) {
            if(left) {
                robot.leftFront.setVelocity(-velocity*2781.1);
                robot.rightFront.setVelocity(velocity*2781.1);
                robot.leftRear.setVelocity(-velocity*2781.1);
                robot.rightRear.setVelocity(velocity*2781.1);
                sleep(time90+offset);
            } else {
                robot.leftFront.setVelocity(velocity*2781.1);
                robot.rightFront.setVelocity(-velocity*2781.1);
                robot.leftRear.setVelocity(velocity*2781.1);
                robot.rightRear.setVelocity(-velocity*2781.1);
                sleep(time90+offset);
            }
        } else if (angle==180) {
            if(left) {
                robot.leftFront.setVelocity(-velocity*2781.1);
                robot.rightFront.setVelocity(velocity*2781.1);
                robot.leftRear.setVelocity(-velocity*2781.1);
                robot.rightRear.setVelocity(velocity*2781.1);
                sleep(time180+offset);
            } else {
                robot.leftFront.setVelocity(velocity*2781.1);
                robot.rightFront.setVelocity(-velocity*2781.1);
                robot.leftRear.setVelocity(velocity*2781.1);
                robot.rightRear.setVelocity(-velocity*2781.1);
                sleep(time180+offset);
            }
        } else if (angle==45) {
            if(left) {
                robot.leftFront.setVelocity(-velocity*2781.1);
                robot.rightFront.setVelocity(velocity*2781.1);
                robot.leftRear.setVelocity(-velocity*2781.1);
                robot.rightRear.setVelocity(velocity*2781.1);
                sleep(time45+offset);
            } else {
                robot.leftFront.setVelocity(velocity*2781.1);
                robot.rightFront.setVelocity(-velocity*2781.1);
                robot.leftRear.setVelocity(velocity*2781.1);
                robot.rightRear.setVelocity(-velocity*2781.1);
                sleep(time45+offset);
            }
        }
    }

    public void end() {
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        sleep(30000);
    }

    public void timeout(long timeout) {
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        sleep(timeout);
    }

    public void lift() {
        liftController.setPIDF(p, i, d, f);
        liftPos = robot.leftLift.getCurrentPosition();
        pidf = liftController.calculate(liftPos, liftTargetPos);
        robot.lift.set(pidf);
    }

    public void extend(int position) {
        encoder.runTo(robot.extend, position);
    }

    public void claw(double position) {
        robot.claw.setPosition(position);
    }

    public void clawWrist(double position) {
        robot.clawWrist.setPosition(position);
    }

    public void clawRotation(double position) {
        robot.clawRotation.setPosition(position);
    }
}
