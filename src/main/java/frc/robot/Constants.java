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
        public static final double flOffsetRot = 0;

        public static final int frDriveId = 3;
        public static final int frSteerId = 4;
        public static final int frEncoderId = 1;
        public static final double frOffsetRot = 0;

        public static final int blDriveId = 5;
        public static final int blSteerId = 6;
        public static final int blEncoderId = 2;
        public static final double blOffsetRot = 0;

        public static final int brDriveId = 7;
        public static final int brSteerId = 8;
        public static final int brEncoderId = 3;
        public static final double brOffsetRot = 0;

        public static final int gyroId = 10;

        public static final double maxDriveMPS = 6;
        public static final double maxSteerRadPS = Math.PI;

        public static final double driveP = 0.5; // V/(M/s)
        public static final double driveI = 0;
        public static final double driveD = 1;
        public static final double driveFF = 0.2;

        public static final double steerP = 1.5; // V/(Rad/s)
        public static final double steerI = 0;
        public static final double steerD = 0.25;
        public static final double steerFF = 0;

        public static final double driveMOI = 0.2;
        public static final double steerMOI = 0.2;

        public static final double deadband = 0.2;
    }

    public static class PhysicalConstants {
        public static final double robotWidthM = Units.inchesToMeters(10);
        public static final double robotLengthM = Units.inchesToMeters(10);
        public static final double wheelDiameterM = Units.inchesToMeters(4);

        public static final double driveGearRatio = 5.36;
        public static final double steerGearRatio = 150.0 / 7.0;

        public static final double driveRotPM = wheelDiameterM * Math.PI / driveGearRatio;

        public static final double steerRotPRad = 2 * Math.PI / steerGearRatio;
    }
}
