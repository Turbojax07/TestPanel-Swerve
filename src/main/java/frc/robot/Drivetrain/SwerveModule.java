package frc.robot.Drivetrain;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;

public class SwerveModule extends SubsystemBase {
    // Module name
    private String name;

    // Motors
    private CANSparkMax drive;
    private CANSparkMax turn;

    // Encoders
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;
    private AnalogEncoder absEncoder;

    // PID Controller
    private SparkPIDController drivePID;
    private SparkPIDController turnPID;

    public SwerveModule(String name, int driveId, int turnId, int absEncoderId, double absEncoderOffset) {
        this.name = name;
        
        // Initializing the motors
        drive = new CANSparkMax(driveId, MotorType.kBrushless);

        // Resetting motor configs
        drive.restoreFactoryDefaults();

        // Inverting motors
        drive.setInverted(true);

        // Setting the idle modes
        drive.setIdleMode(IdleMode.kCoast);

        // Setting current limits
        drive.setSmartCurrentLimit(70);

        // Getting encoders
        driveEncoder = drive.getEncoder();

        // Getting PID Controllers
        drivePID = drive.getPIDController();

        // Setting PID values
        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);



        // Initializing the cancoder
        absEncoder = new AnalogEncoder(absEncoderId);
        absEncoder.setPositionOffset(absEncoderOffset);


        // Setting conversion values for the encoders
        driveEncoder.setPositionConversionFactor(PhysicalConstants.drivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(PhysicalConstants.driveVelocityConversionFactor);

        // Saving the configs for each motor
        drive.burnFlash();

        if (turnId == 8) return;

        turn = new CANSparkMax(turnId, MotorType.kBrushless);
        turn.restoreFactoryDefaults();
        turn.setInverted(false);
        turn.setIdleMode(IdleMode.kCoast);
        turn.setSmartCurrentLimit(30);
        turnEncoder = turn.getEncoder();
        turnEncoder.setPosition(absEncoder.get());
        turnEncoder.setPositionConversionFactor(PhysicalConstants.turnPositionConversionFactor);
        turnEncoder.setVelocityConversionFactor(PhysicalConstants.turnVelocityConversionFactor);
        turnPID = turn.getPIDController();
        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);
        turnPID.setFF(DriveConstants.turnFF);
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnPID.setPositionPIDWrappingMinInput(-Math.PI);
        turn.burnFlash();

    }

    /** Runs every tick that the subsystem exists. */
    public void periodic() {
        SmartDashboard.putNumber(String.format("/%s/ActualSpeed", name), getVelocity());
        SmartDashboard.putNumber(String.format("/%s/ActualAngle", name), getAngle().getDegrees());
        SmartDashboard.putNumber(String.format("/%s/AbsEncoderPos", name), absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(String.format("/%s/AbsEncoderPos - Offset", name), absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset());
    }

    /**
     * Gets the angle of the serve module.
     * 
     * @return The angle of the swerve module as a {@link Rotation2d}.
     */
    public Rotation2d getAngle() {
        if (turnEncoder == null) return new Rotation2d();
        return Rotation2d.fromRadians(turnEncoder.getPosition());
    }

    /**
     * Sets the angle of the swerve module.
     * 
     * @param angle The new angle of the swerve module as a {@link Rotation2d}.
     */
    public void setAngle(Rotation2d angle) {
        if (turnPID == null) return;
        turnPID.setReference(angle.getRadians(), ControlType.kPosition);
    }

    /**
     * Gets the distance the swerve module has traveled.
     * 
     * @return The distance the swerve module has traveled as a double
     */
    public double getDistance() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the velocity of the swerve module in meters per second.
     * 
     * @return The velocity of the swerve module as a double.
     */
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Sets the velocity of the swerve module.
     * 
     * @param speedMetersPerSecond The new velocity of the swerve module as a double.
     */
    public void setVelocity(double speedMetersPerSecond) {
        drivePID.setReference(speedMetersPerSecond, ControlType.kVelocity);
    }

    /**
     * Gets the angle of the absolute encoder with it's offset applied.
     * 
     * @return The angle of the absolute encoder as a {@link Rotation2d}.
     */
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(absEncoder.get());
    }

    /**
     * Gets the angle of the absolute encoder without it's offset.
     * 
     * @return The angle of the absolute encoder as a {@link Rotation2d}.
     */
    public Rotation2d getAbsoluteAngleNoOffset() {
        return Rotation2d.fromRotations(absEncoder.getAbsolutePosition());
    }

    /**
     * Gets the position of the swerve module.
     * 
     * @return The position of the swerve module as a {@link SwerveModulePosition}.
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }
    
    /**
     * Gets the state of the swerve module.
     * 
     * @return The state of the swerve module as a {@link SwerveModuleState}.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAbsoluteAngle());
    }

    /**
     * Sets the state of the swerve module.
     * 
     * @param state The new ideal state of the swerve module.
     */
    public void setState(SwerveModuleState state) {

        SwerveModuleState newState = SwerveModuleState.optimize(state, getAngle());

        SmartDashboard.putNumber(String.format("/%s/DesiredSpeed", name), newState.speedMetersPerSecond);
        SmartDashboard.putNumber(String.format("/%s/DesiredAngle", name), newState.angle.getDegrees());

        // Setting the speed and position of each motor
        setVelocity(newState.speedMetersPerSecond);
        setAngle(newState.angle);
    }
}