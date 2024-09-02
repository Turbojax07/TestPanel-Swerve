package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;

public class SwerveModule extends SubsystemBase {

    /*           backLeft    frontLeft
     *                 x_____x
     *                 |     |
     * --------shooter-----------intake--- x-axis
     *                 |_____|
     *                 x     x
     *           backRight   frontRight
     */
    
    private final String name;

    private final CANSparkMax drive, turn;
    private final RelativeEncoder driveRelEnc, turnRelEnc;
    private final SparkPIDController drivePID, turnPID;

    private final SlewRateLimiter driveAccelerationLimiter = new SlewRateLimiter(6);

    private final AnalogEncoder absEncoder;
    private final double encOffset;

    private SwerveModuleState state = new SwerveModuleState();

    public SwerveModule(String name, int driveID, int turnID, int encID, double encOffset) {

        this.name = name;

        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        absEncoder = new AnalogEncoder(encID);
        this.encOffset = encOffset;

        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        drive.setSmartCurrentLimit(60);
        turn.setSmartCurrentLimit(30);

        drive.setInverted(true);
        turn.setInverted(false);

        drive.setIdleMode(IdleMode.kBrake);
        turn.setIdleMode(IdleMode.kBrake);

        driveRelEnc = drive.getEncoder();
        driveRelEnc.setPosition(0.0);
        driveRelEnc.setPositionConversionFactor(PhysicalConstants.drivePositionConversionFactor);
        driveRelEnc.setVelocityConversionFactor(PhysicalConstants.drivePositionConversionFactor / 60.0);

        turnRelEnc = turn.getEncoder();
        turnRelEnc.setPositionConversionFactor(PhysicalConstants.turnPositionConversionFactor);
        turnRelEnc.setVelocityConversionFactor(PhysicalConstants.turnPositionConversionFactor / 60.0);
        
        drivePID = drive.getPIDController();
        drivePID.setP(DriveConstants.driveP, 0);
        drivePID.setI(DriveConstants.driveI, 0);
        drivePID.setD(DriveConstants.driveD, 0);
        drivePID.setFF(DriveConstants.driveFF, 0);

        turnPID = turn.getPIDController();
        turnPID.setP(DriveConstants.turnP, 0);
        turnPID.setI(DriveConstants.turnI, 0);
        turnPID.setD(DriveConstants.turnD, 0);
        turnPID.setFF(DriveConstants.turnFF, 0);

        // new DebugPID(drivePID, "Drive/Drive");
        // new DebugPID(turnPID, "Drive/Turn");

        drive.burnFlash();
        turn.burnFlash();
    }

    /**
     * Publishes telemetry
     */
    public void periodic() {
        // TODO
        // Telemetry
        // Logger.recordOutput(name + "/Drive/MotorCurrent", drive.getOutputCurrent());
        // Logger.recordOutput(name + "/Drive/BusVoltage", drive.getBusVoltage());
        // Logger.recordOutput(name + "/Drive/Velocity", driveRelEnc.getVelocity());
        // Logger.recordOutput(name + "/Drive/Temperature", drive.getMotorTemperature());

        // Logger.recordOutput(name + "/Turn/MotorCurrent", turn.getOutputCurrent());
        // Logger.recordOutput(name + "/Turn/BusVoltage", turn.getBusVoltage());
        // Logger.recordOutput(name + "/Turn/Position", turnRelEnc.getPosition());
        // Logger.recordOutput(name + "/Turn/Temperature", turn.getMotorTemperature());

        SmartDashboard.putNumber(name + "/AbsPosition", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(name + "/TurnAngle", getHeading());
        SmartDashboard.putNumber(name + "/WheelSpeed", driveRelEnc.getVelocity());
        SmartDashboard.putNumber(name + "/StateMS", state.speedMetersPerSecond);
        SmartDashboard.putNumber(name + "/CurrentDraw", drive.getOutputCurrent());
    }

    /**
     * Sets the relative encoder to the value measured by the absolute encoder
     */
    public void initializeEncoder() {
        turnRelEnc.setPosition((absEncoder.getAbsolutePosition() - encOffset) * (2.0 * Math.PI));
    }

    /**
     * Used to enable continuous motion (prevents the module from spinning 360 degrees going from -180 to 180)
     * @param angle The desired angle of the module
     * @return The input angle to the turn PID
     */
    public double getAdjustedAngle(double angle) {
        double theta = getHeading() - angle;

        if (theta >= Math.PI) {
            theta-=(2.0 * Math.PI);
        }
        if (theta <= -Math.PI) {
            theta+=(2.0 * Math.PI);
        }

        return turnRelEnc.getPosition() - theta;
    }

    /**
     * @return Returns the current being drawn by the drive motor
     */
    public double getDriveCurrent() {
        return drive.getOutputCurrent();
    }

    public double getDriveFF() {
        return drivePID.getFF();
    }

    /**
     * @return Returns the current heading of the module in cheesians
     */
    public double getHeading() {
        return turnRelEnc.getPosition()%(2.0 * Math.PI);
    }

    /**
     * @return Returns the position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveRelEnc.getPosition(), new Rotation2d(getHeading()));
    }

    /**
     * @return Returns the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveRelEnc.getVelocity(), new Rotation2d(getHeading()));
    }

    /**
     * @return Returns the current being drawn by the turn motor
     */
    public double getTurnCurrent() {
        return turn.getOutputCurrent();
    }

    /**
     * Sets the drive PID FF value
     * @param FF The desired FF value
     */
    public void setDriveFF(double FF) {
        drivePID.setFF(FF);
    }

    public void setRatedState(SwerveModuleState state) {
        this.state = state;
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(getHeading()));

        double desiredAngle = optimizedState.angle.getRadians();
        double adjustedAngle = getAdjustedAngle(desiredAngle);

        turnPID.setReference(adjustedAngle, ControlType.kPosition, 0);
        drivePID.setReference(driveAccelerationLimiter.calculate(optimizedState.speedMetersPerSecond), ControlType.kVelocity);
    }

    /**
     * Sets the desired state of the module with a feedback controller
     * @param state The desired state of the module
     */
    public void setState(SwerveModuleState state) {
        this.state = state;
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(getHeading()));

        double desiredAngle = optimizedState.angle.getRadians();
        double adjustedAngle = getAdjustedAngle(desiredAngle);

        turnPID.setReference(adjustedAngle, ControlType.kPosition, 0);
        drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0);
    }

    /**
     * Stops the turn and drive motors
     */
    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }

}