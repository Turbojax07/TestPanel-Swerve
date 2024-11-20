package frc.robot.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    private GyroIO gyroIO;

    private static Gyro instance;

    public static Gyro getInstance() {
        if (instance == null) {
            // The default instance is a sim instance.
            // This will help prevent errors.

            SmartDashboard.putString("/GYROERROR", "Drivetrain using default instance.");

            instance = new Gyro(new GyroIOSim());
        }

        return instance;
    }

    public static Gyro getInstance(GyroIO gyroIO) {
        // Replaces the instance variable if instance doesn't exist, or if the IO classes are different.
        if (instance == null ||
            !instance.gyroIO.getClass().equals(gyroIO.getClass())) {

            instance = new Gyro(gyroIO);
        }

        return instance;
    }

    public Gyro(GyroIO gyroIO) {
        this.gyroIO = gyroIO;

        gyroIO.resetConfigs();

        gyroIO.setRotation2d(new Rotation2d());
    }

    /**
     * Resets the configuration of the gyro.
     * 
     * Only does stuff on the Pigeon2 interface.
     */
    public void resetConfigs() {
        gyroIO.resetConfigs();
    }

    /**
     * Updates swerve module positions and odometry
     * Publishes telemetry
     */
    public void periodic() {
        gyroIO.updateInputs();
    }

    /**
     * Gets the angle of the gyro.
     * 
     * @return the angle of the gyro as a Rotation2d.
     */
    public Rotation2d getAngle() {
        return gyroIO.getRotation2d();
    }

    /**
     * Sets the angle of the gyro.
     * 
     * @param angle The new angle of the gyro as a Rotation2d.
     */
    public void setAngle(Rotation2d angle) {
        gyroIO.setRotation2d(angle);
    }
}