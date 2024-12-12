package frc.robot.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Gyro.Gyro;

public class Drivetrain extends SubsystemBase {
    private SwerveModuleIO flModuleIO;
    private SwerveModuleIO frModuleIO;
    private SwerveModuleIO blModuleIO;
    private SwerveModuleIO brModuleIO;

    private SwerveDriveKinematics kinematics;
    private SwerveModuleIO[] modules;
    private Field2d field = new Field2d();

    private Gyro gyro;

    private SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];

    private SwerveModuleState[] xStates = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(3.0 * Math.PI / 4.0)),
            new SwerveModuleState(0, new Rotation2d(      Math.PI / 4.0)),
            new SwerveModuleState(0, new Rotation2d(      Math.PI / 4.0)),
            new SwerveModuleState(0, new Rotation2d(3.0 * Math.PI / 4.0)),
    };

    private SwerveDriveOdometry odometry;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            // The default instance is a sim instance.
            // This will help prevent errors.
            
            SmartDashboard.putString("/DRIVETRAINERROR", "Drivetrain using default instance.");

            instance = new Drivetrain(
                new SwerveModuleIOSim("fl", DriveConstants.flDriveId, DriveConstants.frSteerId, DriveConstants.flEncoderId, DriveConstants.flEncoderOffset),
                new SwerveModuleIOSim("fr", DriveConstants.frDriveId, DriveConstants.frSteerId, DriveConstants.frEncoderId, DriveConstants.frEncoderOffset),
                new SwerveModuleIOSim("bl", DriveConstants.blDriveId, DriveConstants.brSteerId, DriveConstants.blEncoderId, DriveConstants.blEncoderOffset),
                new SwerveModuleIOSim("br", DriveConstants.brDriveId, DriveConstants.brSteerId, DriveConstants.brEncoderId, DriveConstants.brEncoderOffset));
        }

        return instance;
    }

    public static Drivetrain getInstance(SwerveModuleIO flModuleIO, SwerveModuleIO frModuleIO, SwerveModuleIO blModuleIO, SwerveModuleIO brModuleIO) {
        // Replaces the instance variable if instance doesn't exist, or if the IO classes are different.
        if (instance == null ||
            !instance.flModuleIO.getClass().equals(flModuleIO.getClass()) ||
            !instance.frModuleIO.getClass().equals(frModuleIO.getClass()) ||
            !instance.blModuleIO.getClass().equals(blModuleIO.getClass()) ||
            !instance.brModuleIO.getClass().equals(brModuleIO.getClass())) {

            instance = new Drivetrain(flModuleIO, frModuleIO, blModuleIO, brModuleIO);
        }

        return instance;
    }

    public Drivetrain(SwerveModuleIO flModuleIO, SwerveModuleIO frModuleIO, SwerveModuleIO blModuleIO, SwerveModuleIO brModuleIO) {
        this.flModuleIO = flModuleIO;
        this.frModuleIO = frModuleIO;
        this.blModuleIO = blModuleIO;
        this.brModuleIO = brModuleIO;
        
        modules = new SwerveModuleIO[] { flModuleIO, frModuleIO, blModuleIO, brModuleIO};
        kinematics = new SwerveDriveKinematics(
                new Translation2d(/* FL */-PhysicalConstants.robotWidth / 2.0,  PhysicalConstants.robotLength / 2.0),
                new Translation2d(/* FR */ PhysicalConstants.robotWidth / 2.0,  PhysicalConstants.robotLength / 2.0),
                new Translation2d(/* BL */-PhysicalConstants.robotWidth / 2.0, -PhysicalConstants.robotLength / 2.0),
                new Translation2d(/* BR */ PhysicalConstants.robotWidth / 2.0, -PhysicalConstants.robotLength / 2.0));

        gyro = Gyro.getInstance();
        gyro.resetConfigs();
        gyro.setAngle(new Rotation2d());

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
            states[i] = modules[i].getState();
        }

        odometry = new SwerveDriveOdometry(kinematics, gyro.getAngle(), positions);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getSpeeds,
                this::drive,
                new HolonomicPathFollowerConfig(DriveConstants.maxDriveSpeed, PhysicalConstants.robotWidth / 2.0, new ReplanningConfig()),
                () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)),
                this);
    }

    /**
     * Updates swerve module positions and odometry
     * Publishes telemetry
     */
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
            states[i] = modules[i].getState();
        }

        odometry.update(gyro.getAngle(), positions);
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putData(field);

        flModuleIO.updateInputs();
        frModuleIO.updateInputs();
        blModuleIO.updateInputs();
        brModuleIO.updateInputs();
    }

    /**
     * Drives the robot at the desired speeds with an overall feedback loop to
     * ensure the speeds are met
     *
     * @param speeds The desired speeds of the drivetrain
     */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);

        if (speeds.omegaRadiansPerSecond == 0) {
            for (int i = 0; i < 4; i++) {
                modules[i].setState(desiredStates[i]);
            }
        } else {
            modules[0].setState(desiredStates[2]); // flm -> frs
            modules[1].setState(desiredStates[3]); // frm -> brs
            modules[2].setState(desiredStates[0]); // blm -> fls
            modules[3].setState(desiredStates[1]); // brm -> bls
        }
    }

    /**
     * @return Returns total current drawn by the drive motors
     */
    public double getDriveCurrent() {
        int currentSum = 0;
        for (SwerveModuleIO m : modules) {
            currentSum += m.getDriveCurrent();
        }

        return currentSum;
    }

    /**
     * @return Returns total current drawn by the turn motors
     */
    public double getSteerCurrent() {
        int sum = 0;
        for (SwerveModuleIO m : modules) {
            sum += m.getSteerCurrent();
        }
        return sum;
    }

    /**
     * @return Returns swerve module instances
     */
    public SwerveModuleIO[] getModules() {
        return modules;
    }

    /**
     * @return Returns the robot position reported by the odometry
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Sets the odometry position and updates the gyro angle
     * 
     * @param pose The position to set odometry to.
     */
    public void setPose(Pose2d pose) {
        field.setRobotPose(pose);
        odometry.resetPosition(gyro.getAngle(), positions, pose);
    }

    /**
     * @return Returns all swerve module positions in order of [FL, FR, BL, BR]
     */
    public SwerveModulePosition[] getPositions() {
        return positions;
    }

    /**
     * @return Returns the chassis speeds reported by the measured module states
     */
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(states);
    }

    /**
     * Sets the modules in an X pattern.
     */
    public void makeX() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(xStates[i]);
        }
    }
}
