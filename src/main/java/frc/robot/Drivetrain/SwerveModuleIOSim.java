package frc.robot.Drivetrain;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.PhysicalConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private String name;

    private FlywheelSim driveSim;

    private TalonFX drive;
    private TalonFX steer;

    private SwerveModuleIOInputs inputs;

    public SwerveModuleIOSim(String name, int driveId, int steerId, int encoderId, Angle encoderOffset) {
        // driveSim.setState(encoderOffset);
        driveSim.setInputVoltage(12);
        driveSim.getAngularVelocityRPM();

        this.name = name;

        // Initializing the motors
        drive = new TalonFX(driveId);
        steer = new TalonFX(steerId);

        inputs = new SwerveModuleIOInputs();
    }

    /** Updates the IO inputs for them to be pushed to NetworkTables. */
    public void updateInputs() {
        inputs.state = getState();
        inputs.position = getPosition();

        inputs.driveTemp = getDriveTemperature();
        inputs.driveVoltage = getDriveVoltage();
        inputs.driveCurrent = getDriveCurrent();

        inputs.steerTemp = getSteerTemperature();
        inputs.steerVoltage = getSteerVoltage();
        inputs.steerCurrent = getSteerCurrent();

        Logger.processInputs(name, inputs);
    }

    /**
     * Returns the current state of the swerve module.
     * 
     * @return The current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getAngle()));
    }

    /**
     * Sets the state of the swerve module.
     * 
     * @param state The new state of the swerve module.
     */
    public void setState(SwerveModuleState state) {
        state.optimize(new Rotation2d(getAngle()));

        setVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
        setAngle(state.angle.getMeasure());
    }

    /**
     * Returns the current position of the swerve module.
     * 
     * @return The current position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), new Rotation2d(getAngle()));
    }

    /** Resets the position of the swerve module to a new instance of {@link SwerveModulePosition}. */
    public void resetPosition() {
        resetDistance();
        resetAngle();
    }

    /**
     * Resets the position of the swerve module to the value defined in the position parameter.
     * 
     * @param position The new position of the swerve module.
     */
    public void resetPosition(SwerveModulePosition position) {
        resetDistance(Meters.of(position.distanceMeters));
        resetAngle(position.angle.getMeasure());
    }

    /**
     * Returns the current distance the swerve module has traveled.
     * 
     * @return The current distance the swerve module has traveled as a {@link Distance}.
     */
    public Distance getDistance() {
        return Meters.of(drive.getPosition().getValue().in(Rotations) * PhysicalConstants.driveRotToMeters);
    }

    /** Resets the distance of the swerve module to 0. */
    public void resetDistance() {
        drive.setPosition(0);
    }

    /**
     * Resets the distance of the swerve module to the value defined in the distance parameter.
     * 
     * @param distance The new distance of the swerve module as a {@link Distance}.
     */
    public void resetDistance(Distance distance) {
        drive.setPosition(distance.in(Meters) / PhysicalConstants.driveRotToMeters);
    }

    /**
     * Returns the current angle of the swerve module.
     * 
     * @return The current angle of the swerve module.
     */
    public Angle getAngle() {
        return steer.getPosition().getValue();
    }

    /**
     * Sets the angle of the swerve module.
     * 
     * @param angle The new desired angle of the swerve module.
     */
    public void setAngle(Angle angle) {
        steer.setControl(new PositionDutyCycle(angle));
    }

    /** Resets the angle of the swerve module to 0. */
    public void resetAngle() {
        steer.setPosition(0);
    }

    /**
     * Resets the angle of the swerve module to the value defined in the angle parameter.
     * 
     * @param angle The new angle of the swerve module as a {@link Angle}.
     */
    public void resetAngle(Angle angle) {
        steer.setPosition(angle);
    }

    /**
     * Returns the current velocity of the swerve module.
     * 
     * @return The current velocity of the swerve module.
     */
    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(drive.getVelocity().getValue().in(RotationsPerSecond) * PhysicalConstants.driveRotToMeters);
    }

    /**
     * Sets the velocity of the swerve module.
     * 
     * @param velocity the new desired velocity of the swerve module.
     */
    public void setVelocity(LinearVelocity velocity) {
        drive.setControl(new VelocityDutyCycle(RotationsPerSecond.of(velocity.in(MetersPerSecond) / PhysicalConstants.driveRotToMeters)));
    }

    /**
     * Returns the current temperature of the drive motor.
     * 
     * @return The current temperature of the drive motor.
     */
    public Temperature getDriveTemperature() {
        return drive.getDeviceTemp().getValue();
    }

    /**
     * Returns the current voltage of the drive motor.
     * 
     * @return The current voltage of the drive motor.
     */
    public Voltage getDriveVoltage() {
        return drive.getMotorVoltage().getValue();
    }

    /**
     * Returns the current of the drive motor.
     * 
     * @return The current of the drive motor.
     */
    public Current getDriveCurrent() {
        return drive.getStatorCurrent().getValue();
    }

    /**
     * Returns the current temperature of the steer motor.
     * 
     * @return The current temperature of the steer motor.
     */
    public Temperature getSteerTemperature() {
        return steer.getDeviceTemp().getValue();
    }

    /**
     * Returns the current voltage of the steer motor.
     * 
     * @return The current voltage of the steer motor.
     */
    public Voltage getSteerVoltage() {
        return steer.getMotorVoltage().getValue();
    }

    /**
     * Returns the current of the steer motor.
     * 
     * @return The current of the steer motor.
     */
    public Current getSteerCurrent() {
        return steer.getStatorCurrent().getValue();
    }
}