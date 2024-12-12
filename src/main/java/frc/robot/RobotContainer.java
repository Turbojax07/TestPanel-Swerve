// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.SwerveModuleIOSim;
import frc.robot.Drivetrain.SwerveModuleIOSparkMax;
import frc.robot.Drivetrain.Commands.SwerveDrive;
import frc.robot.Drivetrain.Commands.YouSpinMeRound;
import frc.robot.Gyro.Gyro;
import frc.robot.Gyro.GyroIOPigeon2;
import frc.robot.Gyro.GyroIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(boolean isReal) {
        // Initializing subsystems
        // Gyro needs to be initialized before Drivetrain or else it won't get the correct gyro.

        if (isReal) {
            Gyro.getInstance(new GyroIOPigeon2(DriveConstants.gyroId));
            Drivetrain.getInstance(
                new SwerveModuleIOSparkMax("FLModule", DriveConstants.flDriveId, DriveConstants.flSteerId, DriveConstants.flEncoderId, DriveConstants.flOffsetRot),
                new SwerveModuleIOSparkMax("BLModule", DriveConstants.blDriveId, DriveConstants.blSteerId, DriveConstants.blEncoderId, DriveConstants.blOffsetRot),
                new SwerveModuleIOSparkMax("FRModule", DriveConstants.frDriveId, DriveConstants.frSteerId, DriveConstants.frEncoderId, DriveConstants.frOffsetRot),
                new SwerveModuleIOSparkMax("BRModule", DriveConstants.brDriveId, DriveConstants.brSteerId, DriveConstants.brEncoderId, DriveConstants.brOffsetRot));
        } else {
            Gyro.getInstance(new GyroIOSim());
            Drivetrain.getInstance(
                new SwerveModuleIOSim("FLModule"),
                new SwerveModuleIOSim("BLModule"),
                new SwerveModuleIOSim("FRModule"),
                new SwerveModuleIOSim("BRModule"));
        }

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        controller.povLeft().toggleOnTrue(new YouSpinMeRound(1));
        controller.povRight().toggleOnTrue(new YouSpinMeRound(-1));
    }

    /**
     * This function is where you set the Autonomous command for the main {@link Robot} class to use.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        return new PrintCommand("no command lol");
        // return new PathPlannerAuto("testauto");
    }

    /**
     * This function is where you set the Teleop command for the {@link Robot} class to use.
     * 
     * @return The command to run in Teleop mode.
     */
    public Command getTeleopCommand() {
        return new SwerveDrive(() -> controller.getLeftX(), () -> -controller.getLeftY(), () -> controller.getRightX());
    }
}
