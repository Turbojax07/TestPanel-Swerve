// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.nio.ByteBuffer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private Command teleopCommand;
    private NetworkTable table;
    private GenericPublisher publisher;

    /**
     * This function is run when the robot is first started up.
     * It runs regardless of whether or not a simulation is active.
     * It should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        Logger.addDataReceiver(new NT4Publisher());

        if (!isSimulation()) {
            Logger.addDataReceiver(new WPILOGWriter());
        }

        if (Constants.replayEnabled) {
            Logger.setReplaySource(new WPILOGReader("log.wpilog"));
        }

        RobotContainer robotContainer = new RobotContainer(isReal());

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
        // if (autonomousCommand != null) {
        //     autonomousCommand.schedule();
        // }
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
        // if (autonomousCommand != null) {
        //     autonomousCommand.cancel();
        // }
    }

    /**
     * This function is called once each time the robot enters Teleop mode.<br>
     * It schedules the command returned by {@link RobotContainer#getTeleopCommand()}.
     */
    @Override
    public void teleopInit() {
        // if (teleopCommand != null) {
        //     teleopCommand.schedule();
        // }

        table = NetworkTableInstance.getDefault().getTable("Test");
        NetworkTableType.getFromString("struct:Pose2d");

        NetworkTablesJNI.publish(table.getTopic("Pose2").getHandle(), NetworkTableType.kRaw.getValue(), "struct:Pose2d", PubSubOption.sendAll(true));

        // NetworkTableValue.makeBoolean(true)
        table.putValue("Pose2d_Type_String", NetworkTableValue.makeString(Pose2d.struct.getTypeString()));
        publisher = table.getTopic("Pose2").genericPublish(Pose2d.struct.getTypeString(), PubSubOption.sendAll(true));
    }

    /** This function is called periodically while the robot is in Teleop mode. */
    @Override
    public void teleopPeriodic() {
        table.putValue("TestValue", NetworkTableValue.makeString("Testing!"));
        
        Pose2d pose = new Pose2d(0, 0, new Rotation2d());
        

        // Logger.recordOutput(null, new Mechanism2d(defaultPeriodSecs, defaultPeriodSecs));
        
        // Getting the byte buffer for a pose2d
        ByteBuffer bb = ByteBuffer.allocate(Pose2d.struct.getSize());

        // Packing the pose into the buffer
        Pose2d.struct.pack(bb, pose);
        publisher.setRaw(bb);

        table.putValue("Pose", NetworkTableValue.makeRaw(bb.array()));
    }

    /**
     * This function is called once each time the robot exits Teleop mode.<br>
     * It cancels the command returned by {@link RobotContainer#getTeleopCommand()}.
     */
    @Override
    public void teleopExit() {
        // if (teleopCommand != null) {
        //     teleopCommand.cancel();
        // }
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