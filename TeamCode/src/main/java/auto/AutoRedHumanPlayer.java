package auto;

import static util.LiftVelocityPIDF.p;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;
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
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
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
public class AutoRedHumanPlayer extends OpMode {

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

    public Pose startPose = new Pose(135.2, 81.8, Math.toRadians(-180));

    public Pose chamberPose1 = new Pose(107.8, 81.8, Math.toRadians(-180));

    public Pose backPose = new Pose(120, 81.8);

    public Pose firstSamplePose = new Pose(84, 117);
    public Pose firstSampleControlPose1 = new Pose(123, 129.7);
    public Pose firstSampleControlPose2 = new Pose(87.2, 92);

    public Pose humanPose1 = new Pose(115, 121.2);

    public Pose secondSamplePose = new Pose(84, 125);
    public Pose secondSampleControlPose1 = new Pose(84, 119);
    public Pose secondSampleControlPose2 = new Pose(84, 119);

    public Pose humanPose2 = new Pose(115, 132, Math.toRadians(-180));

    public Pose intakePose = new Pose(120, 121.2, Math.toRadians(0));

    public Pose chamberPose2 = new Pose(104, 81.8, Math.toRadians(-180)); // 76

    public Pose intakePose2 = new Pose(119, 125.3, Math.toRadians(0));

    public Pose chamberPose3 = new Pose(100, 81.8, Math.toRadians(-180)); // 70.7

    public Pose parkPose = new Pose(123, 123, Math.toRadians(0));

    public Path scorePreload, scoreFirstSpecimen, intake, scoreSecondSpecimen, park;
    public PathChain pushSamples;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(chamberPose1)));
        scorePreload.setConstantHeadingInterpolation(chamberPose1.getHeading());

        pushSamples = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose1), new Point(backPose)))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierCurve(new Point(backPose), new Point(firstSampleControlPose1), new Point(firstSampleControlPose2), new Point(firstSamplePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierLine(new Point(firstSamplePose), new Point(humanPose1)))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierCurve(new Point(humanPose1), new Point(secondSampleControlPose1), new Point(secondSampleControlPose2), new Point(secondSamplePose)))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierLine(new Point(secondSamplePose), new Point(humanPose2)))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierLine(new Point(humanPose2), new Point(intakePose)))
                .setLinearHeadingInterpolation(humanPose2.getHeading(), intakePose.getHeading())

                .build();

        scoreFirstSpecimen = new Path(new BezierCurve(new Point(intakePose), new Point(chamberPose2)));
        scoreFirstSpecimen.setLinearHeadingInterpolation(intakePose.getHeading(), chamberPose2.getHeading());

        intake = new Path(new BezierCurve(new Point(chamberPose2), new Point(intakePose2)));
        intake.setLinearHeadingInterpolation(chamberPose2.getHeading(), intakePose2.getHeading());

        scoreSecondSpecimen = new Path(new BezierCurve(new Point(intakePose2), new Point(chamberPose3)));
        scoreSecondSpecimen.setLinearHeadingInterpolation(intakePose2.getHeading(), chamberPose3.getHeading());

        park = new Path(new BezierLine(new Point(chamberPose3), new Point(parkPose)));
        park.setLinearHeadingInterpolation(chamberPose3.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                claw(CLAW_CLOSED);
                clawRotation(CLAW_ROT_VR);
                clawWrist(CLAW_UP_CHAMBER);
                extend(MEXT);
                liftTargetPos = HIGH_CHAMBER;

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
                    claw(CLAW_OPEN);
                    if(pathTimer.getElapsedTime() >= openClaw) {
                        follower.followPath(pushSamples, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    claw(CLAW_CLOSED);
                    if(pathTimer.getElapsedTime() >= openClaw) {
                        follower.followPath(scoreFirstSpecimen, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    claw(CLAW_OPEN);
                    if(pathTimer.getElapsedTime() >= openClaw) {
                        follower.followPath(intake, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    claw(CLAW_CLOSED);
                    if(pathTimer.getElapsedTime() >= openClaw + 600) {
                        follower.followPath(scoreSecondSpecimen, true);

                        setPathState(5);
                    }
                }
                break;
            case 5:
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

        lift();

        if((pathState == 2 || pathState == 4) && pathTimer.getElapsedTime() >= openClaw+scoreDelay) {
            liftTargetPos = INTAKE;
            clawWrist(CLAW_MID);
            extend(UNEXT);
        }

        if((pathState == 3 || pathState == 5) && pathTimer.getElapsedTime() >= openClaw+scoreDelay) {
            liftTargetPos = HIGH_CHAMBER;
            extend(MEXT);
            clawWrist(CLAW_UP_CHAMBER);
        }

        if(pathState == -1) {
            if(pathTimer.getElapsedTime() >= openClaw+scoreDelay) {
                liftTargetPos = 0;
                extend(UNEXT);
                claw(CLAW_CLOSED);
                clawWrist(CLAW_UP_CHAMBER);
                claw(CLAW_CLOSED);
            } else claw(CLAW_OPEN);
        }

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

