package frc.robot.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
    public class GyroIOInputs implements LoggableInputs {
        public Rotation2d angle;

        @Override
        public void toLog(LogTable table) {
            table.put("Angle", angle);
        }

        @Override
        public void fromLog(LogTable table) {
            angle = table.get("Angle", angle);
        }
    }

    public void updateInputs();
    
    public void resetConfigs();

    public Rotation2d getRotation2d();
    public void setRotation2d(Rotation2d angle);
}