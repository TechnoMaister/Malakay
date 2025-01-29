package util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(group = "util")
public class ServoSlowController extends OpMode {

    public static double increment = 0, targetTime = 0, target = 0;
    ElapsedTime time;

    private Hardware robot;
    private ServoSlow servo;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        servo = new ServoSlow();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Robot Initialized");
    }

    @Override
    public void loop() {
        servo.setPosition(robot.extend, increment, targetTime, target, time);
    }
}
