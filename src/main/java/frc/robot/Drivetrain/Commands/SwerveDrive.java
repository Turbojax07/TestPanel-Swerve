package frc.robot.Drivetrain.Commands;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
        
        double zSpeed = MathUtil.applyDeadband(zSpeedSupplier.get(), 0.2);
        if (zSpeed > 0) zSpeed -= 0.2;
        if (zSpeed < 0) zSpeed += 0.2;
        
        double zRotate = MathUtil.applyDeadband(zRotSupplier.get(), 0.2);
        if (zRotate > 0) zRotate -= 0.2;
        if (zRotate < 0) zRotate += 0.2;
        
        Rotation2d angle = new Rotation2d(Math.atan2(zSpeed, xSpeed)).rotateBy(Rotation2d.fromDegrees(90));
        drivetrain.setStates(0, angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}