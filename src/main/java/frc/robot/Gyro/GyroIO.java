package frc.robot.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public class GyroIOInputs {
        public Rotation2d angle;
    }

    public void updateInputs();
    
    public void resetConfigs();

    public Rotation2d getRotation2d();
    public void setRotation2d(Rotation2d angle);
}