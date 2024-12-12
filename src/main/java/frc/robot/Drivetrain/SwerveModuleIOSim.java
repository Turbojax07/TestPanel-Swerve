package frc.robot.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private String name;

    private FlywheelSim driveSim;
    private FlywheelSim steerSim;

    private PIDController driveController;
    private PIDController steerController;

    private SwerveModuleIOInputsAutoLogged inputs;

    private double driveVoltage;
    private double steerVoltage;

    public SwerveModuleIOSim(String name) {
        this.name = name;

        // Initializing the motors
        driveSim = new FlywheelSim(DCMotor.getNEO(1), PhysicalConstants.driveGearRatio, DriveConstants.driveMOI);
        steerSim = new FlywheelSim(DCMotor.getNEO(1), PhysicalConstants.steerGearRatio, DriveConstants.steerMOI);

        // Initializing the PID controllers
        driveController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
        steerController = new PIDController(DriveConstants.steerP, DriveConstants.steerI, DriveConstants.steerD);

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        driveSim.update(0.02);
        steerSim.update(0.02);

        inputs.state = getState();
        inputs.position = getPosition();

        inputs.driveMeters += driveSim.getAngularVelocityRPM() * PhysicalConstants.driveRotPM / 60 * 0.02;
        inputs.steerRadians += steerSim.getAngularVelocityRadPerSec() * 0.02;
        inputs.driveMPS += driveSim.getAngularVelocityRPM() * PhysicalConstants.driveRotPM / 60;

        inputs.driveCelsius = getDriveTemperature();
        inputs.steerCelsius = getSteerTemperature();

        inputs.driveVoltage = getDriveVoltage();
        inputs.steerVoltage = getSteerVoltage();

        inputs.driveCurrent = getDriveCurrent();
        inputs.steerCurrent = getSteerCurrent();

        Logger.processInputs(name, inputs);
    }

    public void resetAngle() {
        steerSim.setState(steerSim.getAngularVelocityRadPerSec());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.steerRadians);
    }

    public void setAngle(Rotation2d angle) {
        this.steerVoltage = steerController.calculate(getAngle().getRadians(), angle.getRadians());
        steerSim.setInputVoltage(this.steerVoltage);
    }

    public double getVelocity() {
        return inputs.driveMPS;
    }

    public void setVelocity(double mps) {
        this.driveVoltage = driveController.calculate(getVelocity(), mps);
        driveSim.setInputVoltage(this.driveVoltage);
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
        return this.driveVoltage;
    }

    public double getSteerVoltage() {
        return this.steerVoltage;
    }

    public double getDriveCurrent() {
        return driveSim.getCurrentDrawAmps();
    }

    public double getSteerCurrent() {
        return steerSim.getCurrentDrawAmps();
    }
}