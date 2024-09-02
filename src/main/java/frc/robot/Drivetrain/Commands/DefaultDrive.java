package frc.robot.Drivetrain.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class DefaultDrive extends Command {

    private final DoubleSupplier x_trans;
    private final DoubleSupplier y_trans;
    private final DoubleSupplier z_rot;

    private final Drivetrain drivetrain;

    public DefaultDrive(DoubleSupplier x_trans, DoubleSupplier y_trans, DoubleSupplier z_rot) {
        this.x_trans = x_trans;
        this.y_trans = y_trans;
        this.z_rot = z_rot;

        this.drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    public void initialize() {}

    public void execute() {
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            x_trans.getAsDouble() * DriveConstants.maxDriveSpeed,
            y_trans.getAsDouble() * DriveConstants.maxDriveSpeed,
            z_rot.getAsDouble() * DriveConstants.maxTurnSpeed, 
            drivetrain.getAngle()));
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interr) {
        drivetrain.stop();
    }
    
}