// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Drivetrain.Commands.*;
import frc.robot.Drivetrain.Drivetrain;

public class RobotContainer {
    // Initializing the subsystems
    Drivetrain drivetrain = Drivetrain.getInstance();

    // Creating the controllers
    CommandXboxController joystick = new CommandXboxController(0);

    public RobotContainer(boolean isSimulation) {
        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return new PrintCommand("No auto lol");
    }

    public Command getTeleopCommand() {
        return new SwerveDrive(() -> joystick.getLeftX(), () -> joystick.getLeftY(), () -> joystick.getRightX());
    }
}