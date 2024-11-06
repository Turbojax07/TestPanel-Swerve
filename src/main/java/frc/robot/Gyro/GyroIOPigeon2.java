package frc.robot.Gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class GyroIOPigeon2 implements GyroIO {
    private Pigeon2 gyro;

    private GyroIOInputsAutoLogged inputs;

    public GyroIOPigeon2(int gyroId) {
        gyro = new Pigeon2(gyroId);
    }

    @Override
    public void updateInputs() {
        inputs.angle = getRotation2d();

        Logger.processInputs("Gyro", inputs);
    }

    @Override
    public void resetConfigs() {
        gyro.getConfigurator().apply(new Pigeon2Configuration());
    }

    @Override
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    @Override
    public void setRotation2d(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
    }
}