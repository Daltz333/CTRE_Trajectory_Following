// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
  // motor ports
  public static final int kMotorLeftMaster = 0;
  public static final int kMotorLeftFollower = 1;
  public static final int kMotorRightMaster = 2;
  public static final int kMotorRightFollower = 3;

  // drivetrain configurations
  public static final double kDriveOpenLoopRampRate = 0.1; // 0.1s from 0-100%

  // drivetrain characteristics
  public static final double kSensorGearRatio = 1; // Gear ratio between encoder and wheels.
  public static final double kWheelRadiusInches = 3;
  public static final double kCountsPerRev =
      4096; // Encoder counts per revolution of the motor shaft.
  public static final double k100msPerSecond = 10;
  public static final double kGearRatio = 5.4;
  public static final double kMagMultiplier =
      ((kCountsPerRev * kGearRatio) / (Math.PI * (kWheelRadiusInches * 2)));

  public static final double kTrackWidthMeters = 0.546;

  public static final double kS = 1.6993;
  public static final double kV = 2.2741;
  public static final double kA = 1.1605;

  public static final double kP = 3.864;
}
