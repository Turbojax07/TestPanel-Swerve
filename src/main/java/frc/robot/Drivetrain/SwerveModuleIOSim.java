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

        inputs.driveMeters += driveSim.getAngularVelocityRPM() * PhysicalConstants.driveRotToMeters / 60 * 0.02;
        inputs.steerRadians += steerSim.getAngularVelocityRadPerSec() * 0.02;
        inputs.driveMPS += driveSim.getAngularVelocityRPM() * PhysicalConstants.driveRotToMeters / 60;

        inputs.driveCelsius = getDriveTemperature();
        inputs.steerCelsius = getSteerTemperature();

        inputs.driveVoltage = getDriveVoltage();
        inputs.steerVoltage = getSteerVoltage();

        inputs.driveCurrent = getDriveCurrent();
        inputs.steerCurrent = getSteerCurrent();

        Logger.processInputs(name, inputs);
    }

    public void resetAngle() {
        steerSim.setState(0, steerSim.getAngularVelocityRadPerSec());
    }

    public Rotation2d getAngle() {
        return new Rotation2d(steerSim.getAngularPositionRad());
    }

    public void setAngle(Rotation2d angle) {
        steerSim.setInputVoltage(steerController.calculate(getAngle().getRadians(), angle.getRadians()));
    }

    public double getVelocity() {
        return inputs.driveMPS;
    }

    public void setVelocity(double mps) {
        driveSim.setInputVoltage(driveController.calculate(getVelocity(), mps));
    }

    public double getDistance() {
        return inputs.driveMeters;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        setVelocity(optimizedState.speedMetersPerSecond);
        setAngle(optimizedState.angle);
    }

    public double getDriveTemperature() {
        return 0;
    }

    public double getSteerTemperature() {
        return 0;
    }

    public double getDriveVoltage() {
        return getVelocity() / DriveConstants.maxDriveSpeed * RobotController.getInputVoltage();
    }

    public double getSteerVoltage() {
        return steerSim.getAngularVelocityRadPerSec() / DriveConstants.maxSteerSpeed * RobotController.getInputVoltage();
    }

    public double getDriveCurrent() {
        return driveSim.getCurrentDrawAmps();
    }

    public double getSteerCurrent() {
        return steerSim.getCurrentDrawAmps();
    }
}