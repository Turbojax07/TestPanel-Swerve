package frc.robot.Drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.PhysicalConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private String name;

    private FlywheelSim driveSim;

    private TalonFX drive;
    private TalonFX steer;

    private SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModuleIOSim(String name, int driveId, int steerId, int encoderId, double encoderOffset) {
    // driveSim.setState(encoderOffset);
    driveSim.setInputVoltage(12);
    driveSim.getAngularVelocityRPM();

        this.name = name;

        // Initializing the motors
        drive = new TalonFX(driveId);
        steer = new TalonFX(steerId);

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    /** Updates the logged values.  Should be used in the periodic function. */
    @Override
    public void updateInputs() {
        inputs.state = getState();
        inputs.position = getPosition();

        inputs.driveTemp = getDriveTemperature();
        inputs.turnTemp = getTurnTemperature();

        inputs.driveVoltage = getDriveVoltage();
        inputs.turnVoltage = getTurnVoltage();

        inputs.driveCurrent = getDriveCurrent();
        inputs.turnCurrent = getTurnCurrent();

        Logger.processInputs(name, inputs);
    }

    /** Resets the angle of the relative encoder to 0. */
    public void resetAngle() {
        while (steer.setPosition(0) != StatusCode.OK);
    }

    /**
     * Gets the angle of the swerve module.
     * 
     * @return The angle as a Rotation2d.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(steer.getPosition().getValue());
    }

    /**
     * Sets the angle of the swerve module using closed-loop control.
     * 
     * @param angle The angle as a Rotation2d.
     */
    public void setAngle(Rotation2d angle) {
        steer.setControl(new PositionDutyCycle(angle.getRotations()));
    }

    /**
     * Gets the speed of the swerve module.
     * 
     * @return The speed in meters per second.
     */
    public double getVelocity() {
        return drive.getVelocity().getValue() * PhysicalConstants.driveRotToMeters;
    }

    /**
     * Sets the speed of the swerve module.
     * 
     * @param speed The speed in meters per second.
     */
    public void setVelocity(double speed) {
        drive.setControl(new VelocityDutyCycle(speed / PhysicalConstants.driveRotToMeters));
    }

    /**
     * Gets the distance of the swerve module.
     * 
     * @return The distance in meters.
     */
    public double getDistance() {
        return drive.getPosition().getValue() * PhysicalConstants.driveRotToMeters;
    }

    /**
     * Gets the position of the swerve module.
     * 
     * @return The position as a SwerveModulePosition.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    /**
     * Gets the state of the swerve module.
     * 
     * @return The state as a SwerveModuleState.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Sets the state of the swerve module.
     * 
     * @param state The state as a SwerveModuleState.
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        setVelocity(optimizedState.speedMetersPerSecond);
        setAngle(optimizedState.angle);
    }

    @Override
    public double getDriveTemperature() {
        return drive.getDeviceTemp().getValue();
    }

    @Override
    public double getTurnTemperature() {
        return steer.getDeviceTemp().getValue();
    }

    @Override
    public double getDriveVoltage() {
        return drive.getMotorVoltage().getValue();
    }

    @Override
    public double getTurnVoltage() {
        return steer.getMotorVoltage().getValue();
    }

    @Override
    public double getDriveCurrent() {
        return drive.getStatorCurrent().getValue();
    }

    @Override
    public double getTurnCurrent() {
        return steer.getStatorCurrent().getValue();
    }
}