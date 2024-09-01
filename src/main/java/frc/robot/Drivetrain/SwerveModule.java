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
    // Motors
    private CANSparkMax drive;
    private CANSparkMax turn;

    // Encoders
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;
    private AnalogEncoder cancoder;

    // PID Controller
    private SparkPIDController drivePID;
    private SparkPIDController turnPID;

    // Module Vars
    private SwerveModuleState state;
    private SwerveModulePosition position;
    private String name;

    public SwerveModule(String name, int driveID, int turnID, int cancoderID, double offset) {
        // Saving the name of the sparkmax
        this.name = name;

        // Initializing the motors
        drive = new CANSparkMax(driveID, MotorType.kBrushless);

        // Resetting motor configs
        drive.restoreFactoryDefaults();

        // Inverting the motors
        drive.setInverted(true);

        // Setting the neutral mode for the 
        drive.setIdleMode(IdleMode.kCoast);

        drive.setSmartCurrentLimit(70);

        // Getting encoders for the motor
        driveEncoder = drive.getEncoder();

        // Getting PID Controllers for the motor
        drivePID = drive.getPIDController();

        // Setting PID values for each motor
        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);

        // Initializing the cancoder
        cancoder = new AnalogEncoder(cancoderID);
        cancoder.setPositionOffset(offset);

        // Setting conversion values for the encoders
        driveEncoder.setPositionConversionFactor(PhysicalConstants.drivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(PhysicalConstants.driveVelocityConversionFactor);
        

        // Saving the configs for each motor
        // driveConfigurator.apply(driveConfig);
        drive.burnFlash();
  
        this.state = new SwerveModuleState(getVelocity(), getAngle());
        this.position = new SwerveModulePosition(getDistance(), getAngle());

        if (turnID == 8) return;

        turn = new CANSparkMax(turnID, MotorType.kBrushless);
        
        turn.restoreFactoryDefaults();
        
        turn.setInverted(false);
        turn.setIdleMode(IdleMode.kCoast);
        turn.setSmartCurrentLimit(30);

        turnEncoder = turn.getEncoder();
        turnPID = turn.getPIDController();

        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);
        turnPID.setFF(DriveConstants.turnFF);

        turnEncoder.setPosition(cancoder.get());

        turnEncoder.setVelocityConversionFactor(PhysicalConstants.turnVelocityConversionFactor);
        turnEncoder.setPositionConversionFactor(PhysicalConstants.turnPositionConversionFactor);
        
        turn.burnFlash();

    }

    /** Runs every tick that the subsystem exists. */
    public void periodic() {
        // Uncomment to find cancoder offsets
        // setState(new SwerveModuleState(0, new Rotation2d()));
        this.state = new SwerveModuleState(getVelocity(), getAngle());
        this.position = new SwerveModulePosition(getDistance(), getAngle());
        SmartDashboard.putNumber(String.format("/%s/offset", name), cancoder.getPositionOffset());
        SmartDashboard.putNumber(String.format("/%s/ActualSpeed", name), getVelocity());
        SmartDashboard.putNumber(String.format("/%s/ActualAngle", name), getAngle().getDegrees());

        SmartDashboard.putNumber(String.format("/%s/AbsEncoderPos", name), cancoder.getAbsolutePosition());
        SmartDashboard.putNumber(String.format("/%s/AbsEncoderPoswOffset", name), cancoder.get());
    }

    public void resetTurnEncoder() {
        turnEncoder.setPosition(cancoder.get() * PhysicalConstants.turnPositionConversionFactor);
    }

    /**
     * Gets the angle of the serve module.
     * 
     * @return The angle of the swerve module as a Rotation2d.
     */
    public Rotation2d getAngle() {
        if (turnEncoder == null) return new Rotation2d();

        return Rotation2d.fromDegrees(turnEncoder.getPosition());
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
     * Gets the state of the swerve module.
     * 
     * @return the state of the swerve module.
     */
    public SwerveModuleState getState() {
        return this.state;
    }

    /**
     * Gets the position of the swerve module.
     * 
     * @return the position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return this.position;
    }

    /**
     * Gets the velocity of the swerve module.
     * 
     * @return The velocity of the swerve module as a double.
     */
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public void setState(SwerveModuleState state) {
        SmartDashboard.putNumber(String.format("/%s/DesiredSpeed", name), state.speedMetersPerSecond);
        SmartDashboard.putNumber(String.format("/%s/DesiredAngle", name), state.angle.getDegrees());

        // Setting the speed and position of each motor
        drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        if (turnPID == null) return;
        turnPID.setReference(state.angle.getDegrees(), ControlType.kPosition);
    }
}