package util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(group = "util")
public class LiftVelocityPIDF extends OpMode {

    private PIDFController liftController;

    public static double p = 0.003, i = 0.003, d = 0, f = 0.00006;

    public static int target = 0;

   private Hardware robot;

    @Override
    public void init() {
        liftController = new PIDFController(p, i, d, f);

        robot = new Hardware(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        liftController.setPIDF(p, i, d, f);
        int liftPos = robot.leftLift.getCurrentPosition();
        int rightLift = robot.rightLift.getCurrentPosition();
        double pidf = liftController.calculate(liftPos, target);

        robot.lift.set(pidf);

        int error = target - liftPos;
        int left_right_error = liftPos - rightLift;
        int right_error = target - rightLift;

        telemetry.addData("pos", liftPos);
        telemetry.addData("target", target);
        telemetry.addData("error", error);
        telemetry.addData("rightLift", rightLift);
        telemetry.addData("left/right error", left_right_error);
        telemetry.addData("right error", right_error);
        telemetry.update();
    }
}
