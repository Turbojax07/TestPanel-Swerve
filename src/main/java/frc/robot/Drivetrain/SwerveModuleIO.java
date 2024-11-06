package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public double speedMetersPerSecond;
        public double distanceMeters;
        public double angleRadians;

        public double driveTemp;
        public double turnTemp;
        
        public double driveVoltage;
        public double turnVoltage;
        
        public double driveCurrent;
        public double turnCurrent;
    }

    public void updateInputs();

    public SwerveModulePosition getPosition();
    public double getDistance();

    public Rotation2d getAngle();
    public void setAngle(Rotation2d angle);

    public double getSpeed();
    public void setSpeed(double speed);

    public double getDriveCurrent();
    public double getTurnCurrent();

    public double getDriveTemperature();
    public double getTurnTemperature();

    public SwerveModuleState getState();
    public void setState(SwerveModuleState state);
}