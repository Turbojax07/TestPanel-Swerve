package frc.robot.Drivetrain.Commands;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class SwerveDrive extends Command {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zSpeedSupplier;
    private Supplier<Double> zRotSupplier;

    public SwerveDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> zSpeedSupplier, Supplier<Double> zRotSupplier) {
        this.drivetrain = Drivetrain.getInstance();
        this.xSpeedSupplier = xSpeedSupplier;
        this.zSpeedSupplier = zSpeedSupplier;
        this.zRotSupplier = zRotSupplier;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.get(), 0.2);
        if (xSpeed > 0) xSpeed -= 0.2;
        if (xSpeed < 0) xSpeed += 0.2;
        xSpeed *= DriveConstants.maxDriveSpeed;
        
        double zSpeed = MathUtil.applyDeadband(zSpeedSupplier.get(), 0.2);
        if (zSpeed > 0) zSpeed -= 0.2;
        if (zSpeed < 0) zSpeed += 0.2;
        zSpeed *= DriveConstants.maxDriveSpeed;
        
        double zRotate = MathUtil.applyDeadband(zRotSupplier.get(), 0.2);
        if (zRotate > 0) zRotate -= 0.2;
        if (zRotate < 0) zRotate += 0.2;
        zRotate *= DriveConstants.maxTurnSpeed;
        
        drivetrain.drive(new ChassisSpeeds(xSpeed, zSpeed, zRotate));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}