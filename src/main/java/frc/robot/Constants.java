// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final boolean replayEnabled = false;

    public static class DriveConstants {
        public static final int flDriveId = 1;
        public static final int flSteerId = 2;
        public static final int flEncoderId = 0;
        public static final double flEncoderOffset = 0; // Rotations

        public static final int frDriveId = 3;
        public static final int frSteerId = 4;
        public static final int frEncoderId = 1;
        public static final double frEncoderOffset = 0; // Rotations

        public static final int blDriveId = 5;
        public static final int blSteerId = 6;
        public static final int blEncoderId = 2;
        public static final double blEncoderOffset = 0; // Rotations

        public static final int brDriveId = 7;
        public static final int brSteerId = 8;
        public static final int brEncoderId = 3;
        public static final double brEncoderOffset = 0.802; // Rotations

        public static final int gyroId = 10;

        public static final double maxDriveSpeed = 1; // Meters / Second
        public static final double maxSteerSpeed = Math.PI; // Radians / Second

        public static final double driveP = 0.01;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double driveFF = 0.2;

        public static final double steerP = 0.4;
        public static final double steerI = 0;
        public static final double steerD = 0;
        public static final double steerFF = 0;

        public static final double driveMOI = 1;
        public static final double steerMOI = 1;

        public static final double deadband = 0.2;
    }

    public static class PhysicalConstants {
        public static final double robotWidth = Units.inchesToMeters(10); // Meters
        public static final double robotLength = Units.inchesToMeters(10); // Meters
        public static final double wheelDiameter = Units.inchesToMeters(4); // Meters

        public static final double driveGearRatio = 5.36;
        public static final double steerGearRatio = 150.0 / 7.0;

        public static final double driveRotToMeters = wheelDiameter * Math.PI / driveGearRatio; // met/rot

        public static final double steerRotToRad = 2 * Math.PI / steerGearRatio; // rad/rot
    }
}
