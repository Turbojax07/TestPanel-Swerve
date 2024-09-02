// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class DriveConstants {
        public static final int flDriveId = 1;
        public static final int flTurnId = 2;
        public static final int flEncoderId = 0;
        public static final double flEncoderOffset = 0.283; // Rotations

        public static final int frDriveId = 3;
        public static final int frTurnId = 4;
        public static final int frEncoderId = 1;
        public static final double frEncoderOffset = 0.268; // Rotations

        public static final int blDriveId = 5;
        public static final int blTurnId = 6;
        public static final int blEncoderId = 2;
        public static final double blEncoderOffset = 0.306; // Rotations
        
        public static final int brDriveId = 7;
        public static final int brTurnId = 8;
        public static final int brEncoderId = 3;
        public static final double brEncoderOffset = 0.304; // Rotations

        public static final int gyroId = 10;

        public static final double maxDriveSpeed = 2; // Meters / Second
        public static final double maxTurnSpeed = Math.PI / 2; // Radians / Second

        public static final double driveP = 0.2;
        public static final double driveI = 0;
        public static final double driveD = 0.01;
        public static final double driveFF = 0;

        public static final double turnP = 0.2;
        public static final double turnI = 0;
        public static final double turnD = 0;
        public static final double turnFF = 0;

    }

    public static class PhysicalConstants {
        public static final double robotWidth = 10.0 * 0.0254; // Meters
        public static final double robotLength = 10.0 * 0.0254; // Meters
        public static final double wheelDiameter = 4.0 * 0.0254; // Meters

        public static final double driveGearRatio = 5.36;
        public static final double turnGearRatio = 150.0 / 7.0;

        public static final double drivePositionConversionFactor = wheelDiameter * Math.PI / driveGearRatio;
        public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60;

        public static final double turnPositionConversionFactor = 2 * Math.PI / turnGearRatio;
        public static final double turnVelocityConversionFactor = turnPositionConversionFactor / 60;
    }
}