package frc.robot.Drivetrain;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private AnalogEncoder absEncoder;

    // PID Controller
    private SparkPIDController drivePID;
    private SparkPIDController turnPID;

    public SwerveModule(int driveId, int turnId, int absEncoderId, double absEncoderOffset) {
        // Initializing the motors
        drive = new CANSparkMax(driveId, MotorType.kBrushless);
        turn = new CANSparkMax(turnId, MotorType.kBrushless);

        // Resetting motor configs
        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        // Inverting motors
        drive.setInverted(true);
        turn.setInverted(false);

        // Setting the idle modes
        drive.setIdleMode(IdleMode.kCoast);
        turn.setIdleMode(IdleMode.kCoast);

        // Setting current limits
        drive.setSmartCurrentLimit(70);
        turn.setSmartCurrentLimit(30);

        // Getting encoders
        driveEncoder = drive.getEncoder();
        turnEncoder = turn.getEncoder();

        // Getting PID Controllers
        drivePID = drive.getPIDController();
        turnPID = turn.getPIDController();

        // Setting PID values
        drivePID.setP(DriveConstants.driveP);
        drivePID.setI(DriveConstants.driveI);
        drivePID.setD(DriveConstants.driveD);
        drivePID.setFF(DriveConstants.driveFF);

        turnPID.setP(DriveConstants.turnP);
        turnPID.setI(DriveConstants.turnI);
        turnPID.setD(DriveConstants.turnD);
        turnPID.setFF(DriveConstants.turnFF);

        // Initializing the cancoder
        absEncoder = new AnalogEncoder(absEncoderId);
        absEncoder.setPositionOffset(absEncoderOffset);

        turnEncoder.setPosition(absEncoder.get());

        // Setting conversion values for the encoders
        driveEncoder.setPositionConversionFactor(PhysicalConstants.drivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(PhysicalConstants.driveVelocityConversionFactor);
        turnEncoder.setPositionConversionFactor(PhysicalConstants.turnPositionConversionFactor);
        turnEncoder.setVelocityConversionFactor(PhysicalConstants.turnVelocityConversionFactor);

        // Saving the configs for each motor
        drive.burnFlash();
        turn.burnFlash();
    }

    /** Runs every tick that the subsystem exists. */
    public void periodic() {
        SmartDashboard.putNumber("/ActualSpeed", getVelocity());
        SmartDashboard.putNumber("/ActualAngle", getAngle().getDegrees());
        SmartDashboard.putNumber("/AbsEncoderPos", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("/AbsEncoderPos - Offset", absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset());
    }

    /**
     * Gets the angle of the serve module.
     * 
     * @return The angle of the swerve module as a Rotation2d.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnEncoder.getPosition());
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
     * Gets the velocity of the swerve module.
     * 
     * @return The velocity of the swerve module as a double.
     */
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    private Rotation2d getAdjustedAngle(Rotation2d angle) {
        Rotation2d theta = getAngle().minus(angle);

        if (theta.getRadians() >= Math.PI) {
            theta.minus(new Rotation2d(2.0 * Math.PI));
        }
        
        if (theta.getRadians() <= -Math.PI) {
            theta.plus(new Rotation2d(2.0 * Math.PI));
        }

        return getAngle().minus(theta);
    }

    public void setState(SwerveModuleState state) {
        SmartDashboard.putNumber("/DesiredSpeed", state.speedMetersPerSecond);
        SmartDashboard.putNumber("/DesiredAngle", state.angle.getDegrees());

        // Setting the speed and position of each motor
        drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        turnPID.setReference(getAdjustedAngle(state.angle).getRadians(), ControlType.kPosition);
    }
}