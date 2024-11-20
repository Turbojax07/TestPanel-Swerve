package frc.robot.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private String name;

    private DCMotorSim driveSim;
    private DCMotorSim steerSim;

    private PIDController driveController;
    private PIDController steerController;

    private SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModuleIOSim(String name, int driveId, int steerId, int encoderId, double encoderOffset) {
        this.name = name;

        // Initializing the motors
        driveSim = new DCMotorSim(DCMotor.getNEO(1), PhysicalConstants.driveGearRatio, DriveConstants.driveMOI);
        steerSim = new DCMotorSim(DCMotor.getNEO(1), PhysicalConstants.steerGearRatio, DriveConstants.steerMOI);

        // Initializing the PID controllers
        driveController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD, 0.02);
        steerController = new PIDController(DriveConstants.steerP, DriveConstants.steerI, DriveConstants.steerD, 0.02);

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
        steerSim.setState(0, steerSim.getAngularVelocityRadPerSec());
    }

    /**
     * Gets the angle of the swerve module.
     * 
     * @return The angle as a Rotation2d.
     */
    public Rotation2d getAngle() {
        return new Rotation2d(steerSim.getAngularPositionRad());
    }

    /**
     * Sets the angle of the swerve module using closed-loop control.
     * 
     * @param angle The angle as a Rotation2d.
     */
    public void setAngle(Rotation2d angle) {
        steerSim.setInputVoltage(steerController.calculate(getAngle().getRadians(), angle.getRadians()));
    }

    /**
     * Gets the speed of the swerve module.
     * 
     * @return The speed in meters per second.
     */
    public double getVelocity() {
        return driveSim.getAngularVelocityRPM() * PhysicalConstants.driveRotToMeters / 60;
    }

    /**
     * Sets the speed of the swerve module.
     * 
     * @param speed The speed in meters per second.
     */
    public void setVelocity(double speed) {
        driveSim.setInputVoltage(driveController.calculate(getVelocity(), speed));
    }

    /**
     * Gets the distance of the swerve module.
     * 
     * @return The distance in meters.
     */
    public double getDistance() {

        return driveSim.getAngularPositionRotations() * PhysicalConstants.driveRotToMeters;
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
        return 0;
    }

    @Override
    public double getTurnTemperature() {
        return 0;
    }

    @Override
    public double getDriveVoltage() {
        return getVelocity() / DriveConstants.maxDriveSpeed * RobotController.getInputVoltage();
    }

    @Override
    public double getTurnVoltage() {
        return steerSim.getAngularVelocityRadPerSec() / DriveConstants.maxSteerSpeed * RobotController.getInputVoltage();
    }

    @Override
    public double getDriveCurrent() {
        return driveSim.getCurrentDrawAmps();
    }

    @Override
    public double getTurnCurrent() {
        return steerSim.getCurrentDrawAmps();
    }
}