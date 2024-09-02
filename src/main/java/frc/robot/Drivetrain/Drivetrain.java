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
    private SwerveModule flModule;
    private SwerveModule frModule;
    private SwerveModule blModule;
    private SwerveModule brModule;

    // Arrays of data
    private SwerveModule[] modules;
    private SwerveModulePosition[] positions;

    // Gyro
    private Pigeon2 gyro;

    // Kinematics
    private SwerveDriveKinematics kinematics;

    // Odometry / Pose Estimation
    private SwerveDriveOdometry odometry;
    private Pose2d initPose = new Pose2d();
    private Field2d field;

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
        // Initializing the Swerve Modules
        flModule = new SwerveModule("Front Left", DriveConstants.flDriveId, DriveConstants.flTurnId, DriveConstants.flEncoderId, DriveConstants.flEncoderOffset);
        frModule = new SwerveModule("Front Right", DriveConstants.frDriveId, DriveConstants.frTurnId, DriveConstants.frEncoderId, DriveConstants.frEncoderOffset);
        blModule = new SwerveModule("Back Left", DriveConstants.blDriveId, DriveConstants.blTurnId, DriveConstants.blEncoderId, DriveConstants.blEncoderOffset);
        brModule = new SwerveModule("Back Right", DriveConstants.brDriveId, DriveConstants.brTurnId, DriveConstants.brEncoderId, DriveConstants.brEncoderOffset);

        // Initializing the gyro
        gyro = new Pigeon2(DriveConstants.gyroId);

        // Creating the arrays
        modules = new SwerveModule[] {flModule, frModule, blModule, brModule};
        positions = new SwerveModulePosition[] {flModule.getSwerveModulePosition(), frModule.getSwerveModulePosition(), blModule.getSwerveModulePosition(), brModule.getSwerveModulePosition()};

        // Initializing the kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d( PhysicalConstants.robotWidth / 2,  PhysicalConstants.robotLength / 2),
            new Translation2d( PhysicalConstants.robotWidth / 2, -PhysicalConstants.robotLength / 2),
            new Translation2d(-PhysicalConstants.robotWidth / 2,  PhysicalConstants.robotLength / 2),
            new Translation2d(-PhysicalConstants.robotWidth / 2, -PhysicalConstants.robotLength / 2)
        );

        // Initializing the odometry
        odometry = new SwerveDriveOdometry(kinematics, getAngle(), positions, initPose);

        // Initializing the 2d field
        field = new Field2d();
    }

    @Override
    public void periodic() {
        odometry.update(getAngle(), positions);


        System.out.println(flx);



        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putData("/Field", field);
        SmartDashboard.putNumber("Gyro", getAngle().getDegrees());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
        }
    }
}