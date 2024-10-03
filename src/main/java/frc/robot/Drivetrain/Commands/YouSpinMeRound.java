package frc.robot.Drivetrain.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

public class YouSpinMeRound extends Command {
    private Drivetrain drivetrain;
    private int direction;

    public YouSpinMeRound(int direction) {
        drivetrain = Drivetrain.getInstance();
        this.direction = direction;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(0, 0, DriveConstants.maxTurnSpeed * direction));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.makeX();
    }
}