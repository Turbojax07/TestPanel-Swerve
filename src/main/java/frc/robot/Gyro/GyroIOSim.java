package frc.robot.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
    private GyroIOInputs inputs;

    @Override
    public void updateInputs() {
        inputs.angle = getRotation2d();

        Logger.processInputs("Gyro", inputs);
    }

    @Override
    public void resetConfigs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetConfigs'");
    }

    @Override
    public Rotation2d getRotation2d() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotation2d'");
    }

    @Override
    public void setRotation2d(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRotation2d'");
    }
    
}
