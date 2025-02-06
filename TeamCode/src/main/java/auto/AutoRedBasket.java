package auto;

import static util.LiftVelocityPIDF.p;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import util.Encoder;
import util.Hardware;

@Config
@Autonomous(group = "Auto")
public class AutoRedBasket extends OpMode {

    public PoseUpdater poseUpdater;
    public DashboardPoseTracker dashboardPoseTracker;
    public Follower follower;
    public Timer pathTimer, opmodeTimer;
    public double pidf;
    public static double startDelay = 800, scoreDelay = 600, openClaw = 400;
    public int liftPos, liftTargetPos;
    public Hardware robot;
    public Encoder encoder;
    public PIDFController liftController;
    public int pathState;

    public Pose startPose = new Pose(135.5, 39, Math.toRadians(-90));

    public Pose basketPose1 = new Pose(135.5, 20, Math.toRadians(-90));

    public Pose firstSamplePose = new Pose(116.3, 24, Math.toRadians(-180));

    public Pose basketPose2 = new Pose(127, 13.5, Math.toRadians(-45));

    public Pose secondSamplePose = new Pose(116.3, 12.5, Math.toRadians(-180));

    public Pose thirdSamplePose = new Pose(99.8, 21, Math.toRadians(-90));

    public Pose parkPose = new Pose(79.7, 43, Math.toRadians(90));

    public Path scorePreload, getFirstSample, scoreFirstSample, getSecondSample, scoreSecondSample, getThirdSample, scoreThirdSample, park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(basketPose1)));
        scorePreload.setConstantHeadingInterpolation(basketPose1.getHeading());

        getFirstSample = new Path(new BezierLine(new Point(basketPose1), new Point(firstSamplePose)));
        getFirstSample.setLinearHeadingInterpolation(basketPose1.getHeading(), firstSamplePose.getHeading());

        scoreFirstSample = new Path(new BezierLine(new Point(firstSamplePose), new Point(basketPose2)));
        scoreFirstSample.setLinearHeadingInterpolation(firstSamplePose.getHeading(), basketPose2.getHeading());

        getSecondSample = new Path(new BezierLine(new Point(basketPose2), new Point(secondSamplePose)));
        getSecondSample.setLinearHeadingInterpolation(basketPose2.getHeading(), secondSamplePose.getHeading());

        scoreSecondSample = new Path(new BezierLine(new Point(secondSamplePose), new Point(basketPose2)));
        scoreSecondSample.setLinearHeadingInterpolation(secondSamplePose.getHeading(), basketPose2.getHeading());

        getThirdSample = new Path(new BezierLine(new Point(basketPose2), new Point(thirdSamplePose)));
        getThirdSample.setLinearHeadingInterpolation(basketPose2.getHeading(), thirdSamplePose.getHeading());

        scoreThirdSample = new Path(new BezierLine(new Point(thirdSamplePose), new Point(basketPose2)));
        scoreThirdSample.setLinearHeadingInterpolation(thirdSamplePose.getHeading(), basketPose2.getHeading());

        park = new Path(new BezierLine(new Point(basketPose2), new Point(parkPose)));
        park.setLinearHeadingInterpolation(basketPose2.getHeading(), parkPose.getHeading());
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
                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scoreFirstSample, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(getSecondSample, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSecondSample, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(getThirdSample, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scoreThirdSample, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(park, true);
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

        poseUpdater.update();
        dashboardPoseTracker.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
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

        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        poseUpdater.setStartingPose(startPose);

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
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

