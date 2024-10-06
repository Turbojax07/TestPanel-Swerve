package frc.robot.Drivetrain;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import edu.wpi.first.wpilibj.Filesystem;

public class Drivetrain extends SubsystemBase {
    // The YAGSL SwerveDrive controller.
    private SwerveDrive swerveDrive;

    // A common instance of the drivetrain subsystem.
    private static Drivetrain instance;

    /**
     * This function gets a common instance of the drivetrain subsystem that anyone can access.
     * <br>
     * This allows us to not need to pass around subsystems as parameters, and instead run this function whenever we need the subsystem.
     * 
     * @return An instance of the Drivetrain subsystem.
     */
    public static Drivetrain getInstance() {
        // If the instance hasn't been initialized, then initialize it.
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    public Drivetrain() {
        // Initializing the drivetrain
        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(DriveConstants.maxDriveSpeed);
        } catch (IOException err) {
            // If the config directory does not exist, the code fails and the program exits.
            System.out.println("YAGSL config directory not found.  Please redeploy the code.");
            System.exit(1);
        }


        // Enabling heading correction.
        swerveDrive.setHeadingCorrection(true);

        // Discretizing chassis speeds
        swerveDrive.setChassisDiscretization(true, 0.2);

        // Disabling brake mode
        swerveDrive.setMotorIdleMode(false);

        // Setting the max turn speed
        swerveDrive.swerveController.setMaximumAngularVelocity(DriveConstants.maxTurnSpeed);
    }

    public void drive(ChassisSpeeds speeds) {
        swerveDrive.driveFieldOriented(speeds);
    }
}