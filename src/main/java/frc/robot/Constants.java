// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final boolean replayEnabled = false;

    public static class DriveConstants {
        public static final int flDriveId = 1;
        public static final int flTurnId = 2;
        public static final int flEncoderId = 0;
        public static final Angle flEncoderOffset = Rotations.of(0);

        public static final int frDriveId = 3;
        public static final int frTurnId = 4;
        public static final int frEncoderId = 1;
        public static final Angle frEncoderOffset = Rotations.of(0);

        public static final int blDriveId = 5;
        public static final int blTurnId = 6;
        public static final int blEncoderId = 2;
        public static final Angle blEncoderOffset = Rotations.of(0);

        public static final int brDriveId = 7;
        public static final int brTurnId = 8;
        public static final int brEncoderId = 3;
        public static final Angle brEncoderOffset = Rotations.of(0);

        public static final int gyroId = 10;
        
        public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(1);
        public static final AngularVelocity maxTurnSpeed = RadiansPerSecond.of(Math.PI);

        public static final Current driveCurrentLimit = Amps.of(35);
        public static final Current steerCurrentLimit = Amps.of(35);

        public static final double driveP = 0.01;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double driveFF = 0.2;

        public static final double steerP = 0.4;
        public static final double steerI = 0;
        public static final double steerD = 0;
        public static final double steerFF = 0;

        public static final Dimensionless deadband = Percent.of(0.2);
    }

    public static class PhysicalConstants {
        public static final Mass mass = Pounds.of(50); // IDK the weight
        public static final MomentOfInertia inertia = KilogramSquareMeters.of(50); // IDK the inertia
        public static final Distance width = Meters.of(Inches.of(10).in(Meters));
        public static final Distance length = Meters.of(Inches.of(10).in(Meters));
        public static final Distance wheelRadius = Meters.of(Inches.of(2).in(Meters));

        public static final double driveGearRatio = 5.36;
        public static final double turnGearRatio = 150.0 / 7.0;
    
        public static final double driveRotToMeters = 2 * Math.PI * wheelRadius.in(Meters) / driveGearRatio;

        public static final double steerRotToRad = 2 * Math.PI / turnGearRatio;
    }
}
