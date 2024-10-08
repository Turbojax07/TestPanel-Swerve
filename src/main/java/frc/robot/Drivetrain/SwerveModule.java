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
    private String name;

    private CANSparkMax drive, turn;
    private RelativeEncoder driveRelEnc, turnRelEnc;
    private SparkPIDController drivePID, turnPID;

    private SlewRateLimiter driveAccelerationLimiter = new SlewRateLimiter(6);

    private AnalogEncoder absEncoder;
    private double encOffset;

    private SwerveModuleState state = new SwerveModuleState();

    public SwerveModule(String name, int driveID, int turnID, int encID, double encOffset) {

        this.name = name;

        drive = new CANSparkMax(driveID, MotorType.kBrushless);

        absEncoder = new AnalogEncoder(encID);
        this.encOffset = encOffset;

        drive.restoreFactoryDefaults();

        drive.setSmartCurrentLimit(40);

        drive.setInverted(true);

        drive.setIdleMode(IdleMode.kCoast);

        driveRelEnc = drive.getEncoder();
        driveRelEnc.setPosition(0.0);
        driveRelEnc.setPositionConversionFactor(PhysicalConstants.drivePositionConversionFactor);
        driveRelEnc.setVelocityConversionFactor(PhysicalConstants.drivePositionConversionFactor / 60.0);

        drivePID = drive.getPIDController();
        drivePID.setP(DriveConstants.driveP, 0);
        drivePID.setI(DriveConstants.driveI, 0);
        drivePID.setD(DriveConstants.driveD, 0);
        drivePID.setFF(DriveConstants.driveFF, 0);

        drive.burnFlash();

        // Configuring turn motor separately so I can leave out motor 8
        turn = new CANSparkMax(turnID, MotorType.kBrushless);

        turn.restoreFactoryDefaults();

        turn.setSmartCurrentLimit(30);

        turn.setInverted(false);

        turn.setIdleMode(IdleMode.kCoast);

        turnRelEnc = turn.getEncoder();
        turnRelEnc.setPositionConversionFactor(PhysicalConstants.turnPositionConversionFactor);
        turnRelEnc.setVelocityConversionFactor(PhysicalConstants.turnPositionConversionFactor / 60.0);

        turnPID = turn.getPIDController();
        turnPID.setP(DriveConstants.turnP, 0);
        turnPID.setI(DriveConstants.turnI, 0);
        turnPID.setD(DriveConstants.turnD, 0);
        turnPID.setFF(DriveConstants.turnFF, 0);

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

        SmartDashboard.putNumber("/" + name + "/AbsPosition", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("/" + name + "/AbsPoswOffset", absEncoder.getAbsolutePosition() - encOffset);
        SmartDashboard.putNumber("/" + name + "/RealAngle", getHeading().getDegrees());
        SmartDashboard.putNumber("/" + name + "/RealSpeed", getVelocity());
        SmartDashboard.putNumber("/" + name + "/ExpectedAngle", state.angle.getDegrees());
        SmartDashboard.putNumber("/" + name + "/ExpectedSpeed", state.speedMetersPerSecond);
        SmartDashboard.putNumber("/" + name + "/CurrentDraw", drive.getOutputCurrent());
    }

    /**
     * Sets the relative encoder to the value measured by the absolute encoder
     */
    public void initializeEncoder() {
        // turnRelEnc.setPosition((absEncoder.getAbsolutePosition() - encOffset) * (2.0 * Math.PI));
    }

    /**
     * Used to enable continuous motion (prevents the module from spinning 360 degrees going from -180 to 180)
     * @param angle The desired angle of the module
     * @return The input angle to the turn PID
     */
    public double getAdjustedAngle(double angle) {
        double theta = getHeading().getRadians() - angle;

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
    public Rotation2d getHeading() {
        return new Rotation2d(turnRelEnc.getPosition() % (2.0 * Math.PI));
    }

    public void setHeading(Rotation2d angle) {
        turnRelEnc.setPosition(angle.getRadians());
    }

    public double getVelocity() {
        return driveRelEnc.getVelocity();
    }

    /**
     * @return Returns the position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveRelEnc.getPosition(), getHeading());
    }

    /**
     * @return Returns the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveRelEnc.getVelocity(), getHeading());
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
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getHeading());

        double desiredAngle = optimizedState.angle.getRadians();
        double adjustedAngle = desiredAngle;

        drivePID.setReference(driveAccelerationLimiter.calculate(optimizedState.speedMetersPerSecond), ControlType.kVelocity);

        turnPID.setReference(adjustedAngle, ControlType.kPosition, 0);
    }

    /**
     * Sets the desired state of the module with a feedback controller
     * @param state The desired state of the module
     */
    public void setState(SwerveModuleState state) {
        this.state = state;
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getHeading());

        double desiredAngle = optimizedState.angle.getRadians();
        double adjustedAngle = getAdjustedAngle(desiredAngle);

        drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0);
        turnPID.setReference(adjustedAngle, ControlType.kPosition, 0);
    }

    /**
     * Stops the turn and drive motors
     */
    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }
}
