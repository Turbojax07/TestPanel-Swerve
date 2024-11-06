package frc.robot.Drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private String name;
    private TalonFX driveMotor;
    private TalonFX turnMotor;

    private SwerveModuleIOInputsAutoLogged inputs;
    
    public SwerveModuleIOSim(String name, int driveId, int turnId, int encoderId, double encoderOffset) {
        this.name = name;

        driveMotor = new TalonFX(driveId);
        turnMotor = new TalonFX(turnId);

        inputs = new SwerveModuleIOInputsAutoLogged();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    @Override
    public void updateInputs() {
        inputs.speedMetersPerSecond = 0;
        inputs.distanceMeters = 0;
        inputs.angleRadians = 0;
        inputs.driveTemp = 0;
        inputs.turnTemp = 0;
        inputs.driveVoltage = 0;
        inputs.turnVoltage = 0;
        inputs.driveCurrent = 0;
        inputs.turnCurrent = 0;
    }

    @Override
    public double getDistance() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        turnMotor.setPosition(angle.getRotations());
    }

    @Override
    public double getSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpeed'");
    }

    @Override
    public void setSpeed(double speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }

    @Override
    public double getDriveCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveCurrent'");
    }

    @Override
    public double getTurnCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTurnCurrent'");
    }

    @Override
    public void setState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setState'");
    }

    @Override
    public double getDriveTemperature() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveTemperature'");
    }

    @Override
    public double getTurnTemperature() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTurnTemperature'");
    }
}