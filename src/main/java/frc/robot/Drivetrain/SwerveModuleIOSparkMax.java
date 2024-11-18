package frc.robot.Drivetrain;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private String name;

    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkClosedLoopController driveController;

    private SparkMax steerMotor;
    private RelativeEncoder steerEncoder;
    private SparkClosedLoopController steerController;

    private AnalogEncoder absEncoder;

    private SwerveModuleIOInputs inputs;

    public SwerveModuleIOSparkMax(String name, int driveId, int steerId, int encoderId, Angle encoderOffset) {
        this.name = name;

        // Getting the drive motor
        driveMotor = new SparkMax(driveId, MotorType.kBrushless);

        // Getting the drive encoder
        driveEncoder = driveMotor.getEncoder();

        // Making a configuration for the drive motor.
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        // Configuring the drive motor
        driveConfig.apply(
            driveConfig.smartCurrentLimit((int) DriveConstants.driveCurrentLimit.in(Amps))
                .inverted(true)
                .idleMode(IdleMode.kCoast));

        // Configuring the drive PID loop
        driveConfig.closedLoop.apply(
            driveConfig.closedLoop.pidf(
                DriveConstants.driveP,
                DriveConstants.driveI,
                DriveConstants.driveD,
                DriveConstants.driveFF,
                ClosedLoopSlot.kSlot0));

        // Configuring the drive encoder
        driveConfig.encoder.apply(
            driveConfig.encoder.positionConversionFactor(PhysicalConstants.driveRotToMeters)
                .velocityConversionFactor(PhysicalConstants.driveRotToMeters / 60.0));
        
        // Applying the configurations
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        // Getting the steer motor
        steerMotor = new SparkMax(steerId, MotorType.kBrushless);

        // Getting the steer encoder
        steerEncoder = steerMotor.getEncoder();

        // Getting the steer controller
        steerController = steerMotor.getClosedLoopController();

        // Making a configuration for the steer motor.
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        // Configuring the steer motor
        steerConfig.apply(
            steerConfig.smartCurrentLimit((int) DriveConstants.steerCurrentLimit.in(Amps))
                .inverted(false)
                .idleMode(IdleMode.kCoast));

        // Configuring the steer PID loop
        steerConfig.closedLoop.apply(
            steerConfig.closedLoop.pidf(
                DriveConstants.steerP,
                DriveConstants.steerI,
                DriveConstants.steerD,
                DriveConstants.steerFF,
                ClosedLoopSlot.kSlot0));

        // Configuring the steer encoder
        steerConfig.encoder.apply(
            steerConfig.encoder.positionConversionFactor(PhysicalConstants.steerRotToRad)
                .velocityConversionFactor(PhysicalConstants.steerRotToRad / 60));
        
        // Applying the configurations
        steerMotor.configure(steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        // Initializing the analog encoder
        absEncoder = new AnalogEncoder(encoderId);

        // Setting the steer encoder's position to one in this range.
        while (steerEncoder.setPosition((absEncoder.get() - encoderOffset.in(Rotations)) * Math.PI * 2) != REVLibError.kOk);

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
        return Meters.of(driveEncoder.getPosition());
    }

    /** Resets the distance of the swerve module to 0. */
    public void resetDistance() {
        driveEncoder.setPosition(0);
    }

    /**
     * Resets the distance of the swerve module to the value defined in the distance parameter.
     * 
     * @param distance The new distance of the swerve module as a {@link Distance}.
     */
    public void resetDistance(Distance distance) {
        driveEncoder.setPosition(distance.in(Meters) / PhysicalConstants.driveRotToMeters);
    }

    /**
     * Returns the current angle of the swerve module.
     * 
     * @return The current angle of the swerve module.
     */
    public Angle getAngle() {
        return Radians.of(steerEncoder.getPosition());
    }

    /**
     * Sets the angle of the swerve module.
     * 
     * @param angle The new desired angle of the swerve module.
     */
    public void setAngle(Angle angle) {
        steerController.setReference(angle.in(Radians), ControlType.kPosition);
    }

    /** Resets the angle of the swerve module to 0. */
    public void resetAngle() {
        steerEncoder.setPosition(0);
    }

    /**
     * Resets the angle of the swerve module to the value defined in the angle parameter.
     * 
     * @param angle The new angle of the swerve module as a {@link Angle}.
     */
    public void resetAngle(Angle angle) {
        steerEncoder.setPosition(angle.in(Radians));
    }

    /**
     * Returns the current velocity of the swerve module.
     * 
     * @return The current velocity of the swerve module.
     */
    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(driveEncoder.getVelocity());
    }

    /**
     * Sets the velocity of the swerve module.
     * 
     * @param velocity the new desired velocity of the swerve module.
     */
    public void setVelocity(LinearVelocity velocity) {
        driveController.setReference(velocity.in(MetersPerSecond), ControlType.kVelocity);
    }

    /**
     * Returns the current temperature of the drive motor.
     * 
     * @return The current temperature of the drive motor.
     */
    public Temperature getDriveTemperature() {
        return Celsius.of(driveMotor.getMotorTemperature());
    }

    /**
     * Returns the current voltage of the drive motor.
     * 
     * @return The current voltage of the drive motor.
     */
    public Voltage getDriveVoltage() {
        return Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    }

    /**
     * Returns the current of the drive motor.
     * 
     * @return The current of the drive motor.
     */
    public Current getDriveCurrent() {
        return Amps.of(driveMotor.getOutputCurrent());
    }

    /**
     * Returns the current temperature of the steer motor.
     * 
     * @return The current temperature of the steer motor.
     */
    public Temperature getSteerTemperature() {
        return Celsius.of(steerMotor.getMotorTemperature());
    }

    /**
     * Returns the current voltage of the steer motor.
     * 
     * @return The current voltage of the steer motor.
     */
    public Voltage getSteerVoltage() {
        return Volts.of(steerMotor.getAppliedOutput() * steerMotor.getBusVoltage());
    }

    /**
     * Returns the current of the steer motor.
     * 
     * @return The current of the steer motor.
     */
    public Current getSteerCurrent() {
        return Amps.of(steerMotor.getOutputCurrent());
    }
}