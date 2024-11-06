package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
            SmartDashboard.putString("/DRIVETRAINERROR", "Drivetrain using default instance.");
            instance = new Drivetrain(null, null, null, null);
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
        int sum = 0;
        for (SwerveModuleIO m : modules) {
            sum += m.getDriveCurrent();
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
     * 
     * 
     * @return Returns total current drawn by the turn motors
     */
    public double getTurnCurrent() {
        int sum = 0;
        for (SwerveModuleIO m : modules) {
            sum += m.getTurnCurrent();
        }
        return sum;
    }

    /**
     * Sets the odometry position and updates the gyro angle
     * 
     * @param pose The position to set odometry to.
     */
    public void resetPose(Pose2d pose) {
        field.setRobotPose(pose);
        odometry.resetPosition(gyro.getAngle(), positions, pose);
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
