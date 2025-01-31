package auto;

import static util.ExtendVelocityPIDF.D;
import static util.ExtendVelocityPIDF.F;
import static util.ExtendVelocityPIDF.I;
import static util.ExtendVelocityPIDF.P;
import static util.LiftVelocityPIDF.p;
import static util.LiftVelocityPIDF.i;
import static util.LiftVelocityPIDF.d;
import static util.LiftVelocityPIDF.f;
import static util.RobotConstants.CLAW_CLOSED;
import static util.RobotConstants.CLAW_MID;
import static util.RobotConstants.CLAW_OPEN;
import static util.RobotConstants.CLAW_ROT_VR;
import static util.RobotConstants.CLAW_UP_CHAMBER;
import static util.RobotConstants.DOWN;
import static util.RobotConstants.HIGH_CHAMBER;
import static util.RobotConstants.UNEXT2;

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
import util.Hardware2;

@Config
@Autonomous(group = "Auto")
public class Auto2 extends OpMode {

    public Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;
    public double pidf, PIDF;
    public static double startDelay = 750, scoreDelay = 1250, nextStepOfTheOperation = 500;
    public int liftPos, liftTargetPos, extPos, extTargetPos;
    public Hardware2 robot;
    public PIDFController liftController, extendController;
    public int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(135, 81.5, Math.toRadians(-180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(100, 81.5, Math.toRadians(-180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose sample1Pose = new Pose(74, 121, Math.toRadians(-180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose human1Pose = new Pose(129, 128, Math.toRadians(-180));

    private final Pose sample1ControlPose = new Pose(135.5, 106, Math.toRadians(-180));
    private final Pose human1ControlPose = new Pose(85, 124, Math.toRadians(-180));

    private Path scorePreload, goToFirstSample, human1;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goToFirstSample = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(sample1ControlPose), new Point(sample1Pose)));
        goToFirstSample.setLinearHeadingInterpolation(scorePose.getHeading(), sample1Pose.getHeading());

        human1 = new Path(new BezierCurve(new Point(sample1Pose), /* Control Point */ new Point(human1ControlPose), new Point(human1Pose)));
        human1.setLinearHeadingInterpolation(sample1Pose.getHeading(), human1Pose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /*claw(CLAW_CLOSED);
                clawRotation(CLAW_ROT_VR);
                clawWrist(CLAW_UP_CHAMBER);
                extTargetPos = MEXT;
                liftTargetPos = HIGH_CHAMBER;

                if(pathTimer.getElapsedTime() >= startDelay) {

                }*/
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
                    extTargetPos = UNEXT2;
                    if(pathTimer.getElapsedTime() >= scoreDelay) liftTargetPos = DOWN;*/

                    follower.followPath(goToFirstSample,true);
                    //setPathState(2);
                    /*if(pathTimer.getElapsedTime() >= scoreDelay + nextStepOfTheOperation) */
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(human1,true);
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

        extTargetPos = UNEXT2;

        //lift();
        extend();

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

        robot = new Hardware2(hardwareMap);

        liftController = new PIDFController(p, i, d, f);
        extendController = new PIDFController(P, I, D, F);

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

    public void claw(double position) {
        robot.claw.setPosition(position);
    }

    public void clawWrist(double position) {
        robot.clawWrist.setPosition(position);
    }

    public void clawRotation(double position) {
        robot.clawRotation.setPosition(position);
    }

    public void extend() {
        extendController.setPIDF(P, I, D, F);
        extPos = robot.extend.getCurrentPosition();
        PIDF = extendController.calculate(extPos, extTargetPos);
        robot.extend.set(pidf);
    }
}

