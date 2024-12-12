package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public SwerveModuleState state = new SwerveModuleState();
        public SwerveModulePosition position = new SwerveModulePosition();

        public double driveMeters = 0;
        public double driveMPS = 0;
        public double steerRadians = 0;

        public double driveCelsius = 0;
        public double steerCelsius = 0;
        
        public double driveVoltage = 0;
        public double steerVoltage = 0;
        
        public double driveCurrent = 0;
        public double steerCurrent = 0;
    }

    /** Updates the logged inputs with current values. */
    public void updateInputs();

    /** @return The position of the swerve module as a {@link SwerveModulePosition}. */
    public SwerveModulePosition getPosition();

    /** @return The current state of the swerve module as a {@link SwerveModuleState}. */
    public SwerveModuleState getState();

    /** 
     * Sets the state of the swerve module.
     * This will run the {@link SwerveModuleIO#setAngle} and {@link SwerveModuleIO#setVelocity} functions.
     * 
     * @param state The new swerve module state.
     */
    public void setState(SwerveModuleState state);

    /** @return The distance of the drive motor in meters. */
    public double getDistance();

    /** @return The angle of the steer motor as a {@link Rotation2d}. */
    public Rotation2d getAngle();
    
    /**
     * Sets the angle that the swerve module will face.
     * This will use a closed-loop PID controller.
     * 
     * @param angle The new angle as a {@link Rotation2d}.
     */
    public void setAngle(Rotation2d angle);

    /** @return The velocity of the drive motor in meters per second. */
    public double getVelocity();

    /**
     * Sets the speed that the swerve module will drive at.
     * This will use a closed-loop PID controller.
     * 
     * @param mps The new speed in meters per second.
     */
    public void setVelocity(double mps);

    /** @return The temperature of the drive motor in degrees celsius. */
    public double getDriveTemperature();

    /** @return The temperature of the steer motor in degrees celsius. */
    public double getSteerTemperature();

    /** @return The voltage of the drive motor. */
    public double getDriveVoltage();

    /** @return The voltage of the steer motor. */
    public double getSteerVoltage();

    /** @return The current of the drive motor. */
    public double getDriveCurrent();

    /** @return The current of the steer motor. */
    public double getSteerCurrent();
}