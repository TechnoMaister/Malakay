package auto;

import static util.LiftVelocityPIDF.p;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;
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
public class Auto extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public double pidf;
    public static double startDelay = 750, scoreDelay = 1250, nextStepOfTheOperation = 500;
    public int liftPos, liftTargetPos, extTargetPos;
    public Hardware robot;
    public Encoder encoder;
    public PIDFController liftController;
    public int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(134.2, 80.8, Math.toRadians(-180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(100, 80.8, Math.toRadians(-180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose sample1Pose = new Pose(100, 108.8, Math.toRadians(-180));
    private final Pose sample1ControlPose = new Pose(134.2, 96, Math.toRadians(-180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose human1Pose = new Pose(60, 131, Math.toRadians(-180));
    private final Pose human1ControlPose = new Pose(60, 108, Math.toRadians(-180));

    private final Pose push1 = new Pose(131, 131, Math.toRadians(-180));

    private final Pose line5 = new Pose(100, 131, Math.toRadians(-180));

    private final Pose line6 = new Pose(60, 151, Math.toRadians(-180));
    private final Pose line6ctr = new Pose(60, 131, Math.toRadians(-180));

    private final Pose line7 = new Pose(131, 151, Math.toRadians(-180));

    private final Pose line8 = new Pose(120, 130, Math.toRadians(0));
    private final Pose line8ctr = new Pose(60, 108, Math.toRadians(0));

    private final Pose line9 = new Pose(100, 80.8, Math.toRadians(-180));
    private final Pose line9ctr = new Pose(134.2, 80.8, Math.toRadians(-180));

    private Path scorePreload, getSpecimen, scoreFirstSpecimen;
    private PathChain pushSamples;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        getSpecimen = new Path(new BezierCurve(new Point(line7), new Point(line8ctr), new Point(line8)));
        getSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));

        scoreFirstSpecimen = new Path(new BezierCurve(new Point(line8), new Point(line9ctr), new Point(line9)));
        scoreFirstSpecimen.setLinearHeadingInterpolation(line8.getHeading(), line9.getHeading());

        pushSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(sample1ControlPose), new Point(sample1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample1Pose.getHeading())

                .addPath(new BezierCurve(new Point(sample1Pose), new Point(human1ControlPose), new Point(human1Pose)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), human1Pose.getHeading())

                .addPath(new BezierLine(new Point(human1Pose), new Point(push1)))
                .setLinearHeadingInterpolation(human1Pose.getHeading(), push1.getHeading())

                .addPath(new BezierLine(new Point(push1), new Point(line5)))
                .setLinearHeadingInterpolation(push1.getHeading(), line5.getHeading())

                .addPath(new BezierCurve(new Point(line5), new Point(line6ctr), new Point(line6)))
                .setLinearHeadingInterpolation(line5.getHeading(), line6.getHeading())

                .addPath(new BezierLine(new Point(line6), new Point(line7)))
                .setLinearHeadingInterpolation(line6.getHeading(), line7.getHeading())

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /*claw(CLAW_CLOSED);
                clawRotation(CLAW_ROT_VR);
                clawWrist(CLAW_UP_CHAMBER);
                extend(MEXT);
                liftTargetPos = HIGH_CHAMBER;

                if(pathTimer.getElapsedTime() >= startDelay) {

                }*/
                //follower.followPath(scorePreload);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /*claw(CLAW_OPEN);
                    clawWrist(CLAW_MID);
                    extend(UNEXT);
                    if(pathTimer.getElapsedTime() >= scoreDelay) liftTargetPos = DOWN;*/

                    follower.followPath(pushSamples);
                    /*if(pathTimer.getElapsedTime() >= scoreDelay + nextStepOfTheOperation) */
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(getSpecimen);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(scoreFirstSpecimen);
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
        extend(UNEXT);

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
        follower.setStartingPose(startPose);
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

