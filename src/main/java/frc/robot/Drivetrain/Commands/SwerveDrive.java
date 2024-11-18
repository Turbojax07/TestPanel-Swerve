package frc.robot.Drivetrain.Commands;

import java.util.function.Supplier;
import frc.robot.Drivetrain.SwerveModuleIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

import static edu.wpi.first.units.Units.*;

public class SwerveDrive extends Command {
    private final Supplier<Double> x_trans;
    private final Supplier<Double> y_trans;
    private final Supplier<Double> z_rotat;

    private final Drivetrain drivetrain;

    public SwerveDrive(Supplier<Double> x_trans, Supplier<Double> y_trans, Supplier<Double> z_rot) {
        this.x_trans = x_trans;
        this.y_trans = y_trans;
        this.z_rotat = z_rot;

        this.drivetrain = Drivetrain.getInstance();
        for (SwerveModuleIO module : drivetrain.getModules()) {
            module.resetAngle();
        }

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Getting the inputs
        double xSpeed = x_trans.get();
        double ySpeed = y_trans.get();
        double zRotat = z_rotat.get();

        // Outputting the raw controller values to SmartDashboard
        SmartDashboard.putNumber("/Controller/LeftX_Raw", xSpeed);
        SmartDashboard.putNumber("/Controller/LeftY_Raw", ySpeed);
        SmartDashboard.putNumber("/Controller/RightX_Raw", zRotat);

        // Applying deadband and max speed to the xSpeed input
        xSpeed = MathUtil.applyDeadband(-xSpeed, DriveConstants.deadband.in(Percent));
        if (xSpeed > 0) xSpeed -= DriveConstants.deadband.in(Percent);
        if (xSpeed < 0) xSpeed += DriveConstants.deadband.in(Percent);
        xSpeed *= DriveConstants.maxDriveSpeed.in(MetersPerSecond);

        // Applying deadband and max speed to the ySpeed input
        ySpeed = MathUtil.applyDeadband(-ySpeed, DriveConstants.deadband.in(Percent));
        if (ySpeed > 0) ySpeed -= DriveConstants.deadband.in(Percent);
        if (ySpeed < 0) ySpeed += DriveConstants.deadband.in(Percent);
        ySpeed *= DriveConstants.maxDriveSpeed.in(MetersPerSecond);

        // Applying deadband and max speed to the zRotat input
        zRotat = MathUtil.applyDeadband(-zRotat, DriveConstants.deadband.in(Percent));
        if (zRotat > 0) zRotat -= DriveConstants.deadband.in(Percent);
        if (zRotat < 0) zRotat += DriveConstants.deadband.in(Percent);
        zRotat *= DriveConstants.maxTurnSpeed.in(RadiansPerSecond);

        // Outputting the adjusted controller values to SmartDashboard
        SmartDashboard.putNumber("/Controller/LeftX_Adjusted", xSpeed);
        SmartDashboard.putNumber("/Controller/LeftY_Adjusted", ySpeed);
        SmartDashboard.putNumber("/Controller/RightX_Adjusted", zRotat);

        // Creating the ChassisSpeeds object.
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, zRotat);

        // Driving the robot
        drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
