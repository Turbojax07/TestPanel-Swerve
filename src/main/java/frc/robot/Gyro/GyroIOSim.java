package frc.robot.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
    private GyroIOInputsAutoLogged inputs;
    private Rotation2d angle;

    public GyroIOSim() {
        inputs = new GyroIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        inputs.angle = getRotation2d();

        Logger.processInputs("Gyro", inputs);
    }

    @Override
    public void resetConfigs() {}

    @Override
    public Rotation2d getRotation2d() {
        return angle;
    }

    @Override
    public void setRotation2d(Rotation2d angle) {
        this.angle = angle;
    }
}