package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

    public SwerveDrive() {}

    public void drive(double x1, double y1, double rotation) {
        double r = Math.sqrt((Constants.swervelength * Constants.swervelength) + (Constants.swervewidth * Constants.swervewidth));
        y1 *= -1;
        double motorspeed1 = x1 - rotation * (Constants.swervelength / r);
        double motorspeed2 = x1 + rotation * (Constants.swervelength / r);
        double motorspeed3 = y1 - rotation * (Constants.swervewidth / r);
        double motorspeed4 = y1 - rotation * (Constants.swervewidth / r);
        double backRightSpeed = Math.sqrt ((motorspeed1 * motorspeed1) + (motorspeed4 * motorspeed4));
        double backLeftSpeed = Math.sqrt ((motorspeed1 * motorspeed1) + (motorspeed3 * motorspeed3));
        double frontRightSpeed = Math.sqrt ((motorspeed2 * motorspeed2) + (motorspeed4 * motorspeed4));
        double frontLeftSpeed = Math.sqrt ((motorspeed2 * motorspeed2) + (motorspeed3 * motorspeed3));
        double backRightAngle = Math.atan2 (motorspeed1, motorspeed4) / Math.PI;
        double backLeftAngle = Math.atan2 (motorspeed1, motorspeed3) / Math.PI;
        double frontRightAngle = Math.atan2 (motorspeed2, motorspeed4) / Math.PI;
        double frontLeftAngle = Math.atan2 (motorspeed2, motorspeed3) / Math.PI;
    }
}
