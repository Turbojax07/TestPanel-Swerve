// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Commands.DefaultDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(boolean isSimulation) {
        // Initializing subsystems
        Drivetrain.getInstance();

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
    private void configureBindings() {}

    /**
     * This function is where you set the Autonomous command for the main {@link Robot} class to use.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getAutonomousCommand() {
        return new PrintCommand("No auto lol");
    }

    /**
     * This function is where you set the Teleop command for the {@link Robot} class to use.
     * 
     * @return The command to run in Teleop mode.
     */
    public Command getTeleopCommand() {
        return new DefaultDrive(() -> controller.getLeftX(), () -> controller.getLeftY(), () -> controller.getRightX());
    }
}