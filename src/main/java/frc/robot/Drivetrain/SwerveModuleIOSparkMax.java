package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private String name;

    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkPIDController driveController;

    private CANSparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private SparkPIDController turnController;

    private AnalogEncoder absEncoder;

    private SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModuleIOSparkMax(String name, int driveId, int turnId, int encoderId, double encoderOffset) {
        this.name = name;

        // Configuring the drive motor
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        while (driveMotor.restoreFactoryDefaults() != REVLibError.kOk);
        while (driveMotor.setSmartCurrentLimit(60) != REVLibError.kOk);
        driveMotor.setInverted(true);
        while (driveMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk);

        // Configuring the drive encoder
        driveEncoder = driveMotor.getEncoder();
        while (driveEncoder.setPositionConversionFactor(PhysicalConstants.driveRotPM) != REVLibError.kOk);
        while (driveEncoder.setVelocityConversionFactor(PhysicalConstants.driveRotPM / 60) != REVLibError.kOk);

        // Configuring the drive PID controller
        driveController = driveMotor.getPIDController();
        while (driveController.setP(DriveConstants.driveP) != REVLibError.kOk);
        while (driveController.setI(DriveConstants.driveI) != REVLibError.kOk);
        while (driveController.setD(DriveConstants.driveD) != REVLibError.kOk);
        while (driveController.setFF(DriveConstants.driveFF) != REVLibError.kOk);

        // Saving configs for the drive motor
        while (driveMotor.burnFlash() != REVLibError.kOk);

        // Configuring the turn motor
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        while (turnMotor.restoreFactoryDefaults() != REVLibError.kOk);
        while (turnMotor.setSmartCurrentLimit(35) != REVLibError.kOk);
        turnMotor.setInverted(false);
        while (turnMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk);

        // Configuring the turn encoder
        turnEncoder = turnMotor.getEncoder();
        while (turnEncoder.setPositionConversionFactor(PhysicalConstants.steerRotPRad) != REVLibError.kOk);
        while (turnEncoder.setVelocityConversionFactor(PhysicalConstants.steerRotPRad / 60) != REVLibError.kOk);

        // Confuguring the turn PID controller
        turnController = turnMotor.getPIDController();
        while (turnController.setP(DriveConstants.steerP) != REVLibError.kOk);
        while (turnController.setI(DriveConstants.steerI) != REVLibError.kOk);
        while (turnController.setD(DriveConstants.steerD) != REVLibError.kOk);
        while (turnController.setFF(DriveConstants.steerFF) != REVLibError.kOk);

        // Saving configs for the turn motor
        while (turnMotor.burnFlash() != REVLibError.kOk);

        // Initializing the analog encoder
        absEncoder = new AnalogEncoder(encoderId);

        // Setting the turn encoder's position to one in this range.
        while (turnEncoder.setPosition((absEncoder.get() - encoderOffset) * 2 * Math.PI) != REVLibError.kOk);

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public void updateInputs() {
        inputs.state = getState();
        inputs.position = getPosition();

        inputs.driveCelsius = getDriveTemperature();
        inputs.steerCelsius = getSteerTemperature();

        inputs.driveVoltage = getDriveVoltage();
        inputs.steerVoltage = getSteerVoltage();

        inputs.driveCurrent = getDriveCurrent();
        inputs.steerCurrent = getSteerCurrent();

        Logger.processInputs(name, inputs);
    }

    public void resetAngle() {
        while (turnEncoder.setPosition(0) != REVLibError.kOk);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(turnEncoder.getPosition());
    }

    public void setAngle(Rotation2d angle) {
        while (turnController.setReference(angle.getRadians(), ControlType.kPosition) != REVLibError.kOk);
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public void setVelocity(double speed) {
        while (driveController.setReference(speed, ControlType.kVelocity) != REVLibError.kOk);
    }

    public double getDistance() {
        return driveEncoder.getPosition();
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
        return driveMotor.getMotorTemperature();
    }

    public double getSteerTemperature() {
        return turnMotor.getMotorTemperature();
    }

    public double getDriveVoltage() {
        return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    }

    public double getSteerVoltage() {
        return turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    }

    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public double getSteerCurrent() {
        return turnMotor.getOutputCurrent();
    }
}