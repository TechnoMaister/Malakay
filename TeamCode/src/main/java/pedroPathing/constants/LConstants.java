package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .0007;
        TwoWheelConstants.strafeTicksToInches = .0007;
        TwoWheelConstants.forwardY = 4.52755906;
        TwoWheelConstants.strafeX = -7.48031496;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "forwardEncoder";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "strafeEncoder";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
}




