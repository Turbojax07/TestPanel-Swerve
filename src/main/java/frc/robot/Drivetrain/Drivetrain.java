package frc.robot.Drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;

public class Drivetrain extends SubsystemBase {
    // Swerve Modules
    private SwerveModule module;

    // A common instance of the drivetrain subsystem.
    private static Drivetrain instance;

    /**
     * This function gets a common instance of the drivetrain subsystem that anyone
     * can access.
     * <p>
     * This allows us to not need to pass around subsystems as parameters, and
     * instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Drivetrain subsystem.
     */
    public static Drivetrain getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null)
            instance = new Drivetrain();

        return instance;
    }

    public Drivetrain() {
        // Initializing the Swerve Modules
        module = new SwerveModule(DriveConstants.flDriveId, DriveConstants.flTurnId, DriveConstants.flEncoderId, DriveConstants.flEncoderOffset);
    }

    public void setState(double speed, Rotation2d angle) {
        SwerveModuleState state = new SwerveModuleState(speed, angle);
        module.setState(state);
    }
}