package frc.robot.Drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveModuleIO {
    public class SwerveModuleIOInputs implements LoggableInputs {
        public SwerveModuleState state;
        public SwerveModulePosition position;

        public Temperature driveTemp;
        public Voltage driveVoltage;
        public Current driveCurrent;

        public Temperature steerTemp;
        public Voltage steerVoltage;
        public Current steerCurrent;

        @Override
        public void toLog(LogTable table) {
            table.put(   "State",    state);
            table.put("Position", position);
            
            table.put(   "DriveTemp",    driveTemp);
            table.put("DriveVoltage", driveVoltage);
            table.put("DriveCurrent", driveCurrent);

            table.put(   "SteerTemp",    steerTemp);
            table.put("SteerVoltage", steerVoltage);
            table.put("SteerCurrent", steerCurrent);
        }

        @Override
        public void fromLog(LogTable table) {
            state        = table.get(   "State",    state);
            position     = table.get("Position", position);

            driveTemp    = table.get(   "DriveTemp",    driveTemp);
            driveVoltage = table.get("DriveVoltage", driveVoltage);
            driveCurrent = table.get("DriveCurrent", driveCurrent);

            steerTemp    = table.get(   "SteerTemp",    steerTemp);
            steerVoltage = table.get("SteerVoltage", steerVoltage);
            steerCurrent = table.get("SteerCurrent", steerCurrent);
        }
    }

    public void updateInputs();

    public SwerveModuleState getState();
    public void setState(SwerveModuleState state);

    public SwerveModulePosition getPosition();
    public void resetPosition();
    public void resetPosition(SwerveModulePosition position);

    public Distance getDistance();
    public void resetDistance();
    public void resetDistance(Distance distance);

    public Angle getAngle();
    public void setAngle(Angle angle);
    public void resetAngle();
    public void resetAngle(Angle angle);

    public LinearVelocity getVelocity();
    public void setVelocity(LinearVelocity speed);

    public Temperature getDriveTemperature();
    public Voltage getDriveVoltage();
    public Current getDriveCurrent();

    public Temperature getSteerTemperature();
    public Voltage getSteerVoltage();
    public Current getSteerCurrent();
}