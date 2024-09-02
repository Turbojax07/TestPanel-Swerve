// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private Command teleopCommand;

    /**
     * This function is run when the robot is first started up.
     * It runs regardless of whether or not a simulation is active.
     * It should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotContainer robotContainer = new RobotContainer(isSimulation());

        autonomousCommand = robotContainer.getAutonomousCommand();
        teleopCommand = robotContainer.getTeleopCommand();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically while the robot is in Disabled mode. */
    @Override
    public void disabledPeriodic() {}
    
    /** This function is called once each time the robot exits Disabled mode. */
    @Override
    public void disabledExit() {}

    /**
     * This function is called once each time the robot enters Autonomous mode.<br>
     * It schedules the command returned by {@link RobotContainer#getAutonomousCommand()}.
     */
    @Override
    public void autonomousInit() {
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically while the robot is in Autonomous mode. */
    @Override
    public void autonomousPeriodic() {}

    /**
     * This function is called once each time the robot exits Autonomous mode.<br>
     * It cancels the command returned by {@link RobotContainer#getAutonomousCommand()}.
     */
    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called once each time the robot enters Teleop mode.<br>
     * It schedules the command returned by {@link RobotContainer#getTeleopCommand()}.
     */
    @Override
    public void teleopInit() {
        if (teleopCommand != null) {
            teleopCommand.schedule();
        }
    }

    /** This function is called periodically while the robot is in Teleop mode. */
    @Override
    public void teleopPeriodic() {}

    /**
     * This function is called once each time the robot exits Teleop mode.<br>
     * It cancels the command returned by {@link RobotContainer#getTeleopCommand()}.
     */
    @Override
    public void teleopExit() {
        if (teleopCommand != null) {
            teleopCommand.cancel();
        }
    }

    /** This function is called once each time the robot enters Test mode. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically while the robot is in Test mode. */
    @Override
    public void testPeriodic() {}
    
    /** This function is called once each time the robot exits Test mode. */
    @Override
    public void testExit() {}

    /** This function is called once when the robot is first started up in a simulation. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in a simulation. */
    @Override
    public void simulationPeriodic() {}
}