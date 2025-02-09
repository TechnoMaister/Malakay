package util;

import static util.RobotConstants.liftPIDFCoefficients;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(group = "util")
public class LiftVelocityPIDF extends OpMode {

    public PIDFController controller;
    public static int target = 0;
    int liftPos;
    double power;
   private Hardware robot;

    @Override
    public void init() {
        controller = new PIDFController(liftPIDFCoefficients);

        robot = new Hardware(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        controller.reset();
        liftPos = robot.leftLift.getCurrentPosition();
        controller.updateError(target - liftPos);
        power = controller.runPIDF();
        for (DcMotorEx motor : robot.lift) {
            motor.setPower(power);
        }

        telemetry.addData("pos", liftPos);
        telemetry.addData("target", target);
        telemetry.addData("error", target - liftPos);
        telemetry.addData("power", power);
        telemetry.update();
    }

}
