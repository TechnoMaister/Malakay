package util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(group = "util")
public class ExtendVelocityPIDF extends OpMode {

    private PIDFController controller;

    public static double P = 0.0017, I = 0.001, D = 0.00001, F = 0.00001;

    public static int target = 0;

    private Hardware2 robot;

    @Override
    public void init() {
        controller = new PIDFController(P, I, D, F);

        robot = new Hardware2(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        controller.setPIDF(P, I, D, F);
        int extendPos = robot.extend.getCurrentPosition();
        double pidf = controller.calculate(extendPos, target);

        robot.extend.set(pidf);

        int error = target - extendPos;

        telemetry.addData("pos", extendPos);
        telemetry.addData("target", target);
        telemetry.addData("error", error);
        telemetry.update();
    }
}
