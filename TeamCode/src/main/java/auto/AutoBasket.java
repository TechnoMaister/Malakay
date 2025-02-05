package auto;

import static util.LiftVelocityPIDF.p;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;
import static util.LiftVelocityPIDF.target;
import static util.RobotConstants.CLAW_CLOSED;
import static util.RobotConstants.CLAW_MID;
import static util.RobotConstants.CLAW_OPEN;
import static util.RobotConstants.CLAW_ROT_VR;
import static util.RobotConstants.CLAW_UP_CHAMBER;
import static util.RobotConstants.HIGH_CHAMBER;
import static util.RobotConstants.INTAKE;
import static util.RobotConstants.MEXT;
import static util.RobotConstants.UNEXT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import util.Encoder;
import util.Hardware;

@Config
@Autonomous(group = "Auto")
public class AutoBasket extends OpMode {

    public Follower follower;
    public Timer pathTimer, opmodeTimer;
    public double pidf;
    public static double startDelay = 800, scoreDelay = 600, openClaw = 400;
    public int liftPos, liftTargetPos;
    public Hardware robot;
    public Encoder encoder;
    public PIDFController liftController;
    public int pathState;

    private final Pose startPoint = new Pose(133.8, 38.2, Math.toRadians(-90));

    private final Pose line1 = new Pose(133.8, 23);

    private final Pose line2 = new Pose(98, 30);

    private final Pose line3 = new Pose(98, 20);

    private final Pose line4 = new Pose(98, 10);

    private Path scorePreload, getSample1, scoreSample1, getSample2, scoreSample2, getSample3, scoreSample3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPoint), new Point(line1)));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(-90));

        getSample1 = new Path(new BezierCurve(new Point(line1), new Point(startPoint), new Point(line2)));
        getSample1.setConstantHeadingInterpolation(Math.toRadians(-90));

        scoreSample1 = new Path(new BezierCurve(new Point(line2), new Point(startPoint), new Point(line1)));
        scoreSample1.setConstantHeadingInterpolation(Math.toRadians(-90));

        getSample2 = new Path(new BezierCurve(new Point(line1), new Point(startPoint), new Point(line3)));
        getSample2.setConstantHeadingInterpolation(Math.toRadians(-90));

        scoreSample2 = new Path(new BezierCurve(new Point(line3), new Point(startPoint), new Point(line1)));
        scoreSample2.setConstantHeadingInterpolation(Math.toRadians(-90));

        getSample3 = new Path(new BezierCurve(new Point(line1), new Point(line2), new Point(line4)));
        getSample3.setConstantHeadingInterpolation(Math.toRadians(-90));

        scoreSample3 = new Path(new BezierCurve(new Point(line4), new Point(line2), new Point(line1)));
        scoreSample3.setConstantHeadingInterpolation(Math.toRadians(-90));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(pathTimer.getElapsedTime() >= startDelay) {
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                }
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                if(!follower.isBusy()) {
                    follower.followPath(getSample1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSample1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(getSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSample2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(getSample3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSample3, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        //lift();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPoint);
        buildPaths();

        robot = new Hardware(hardwareMap);
        encoder = new Encoder();

        liftController = new PIDFController(p, i, d, f);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
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

