package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public SwerveModuleState state;
        public SwerveModulePosition position;

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

    public double getVelocity();
    public void setVelocity(double speed);

    public double getDriveTemperature();
    public double getTurnTemperature();

    public double getDriveVoltage();
    public double getTurnVoltage();

    public double getDriveCurrent();
    public double getTurnCurrent();

    public SwerveModuleState getState();
    public void setState(SwerveModuleState state);
}