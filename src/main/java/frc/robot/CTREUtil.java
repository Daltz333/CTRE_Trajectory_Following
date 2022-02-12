// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class CTREUtil {
  public static int distanceToNativeUnits(double positionMeters) {
    double wheelRotations =
        positionMeters / (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
    double motorRotations = wheelRotations * Constants.kSensorGearRatio;
    int sensorCounts = (int) (motorRotations * Constants.kCountsPerRev);
    return sensorCounts;
  }

  public static int velocityToNativeUnits(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond =
        velocityMetersPerSecond
            / (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / Constants.k100msPerSecond;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * Constants.kCountsPerRev);
    return sensorCountsPer100ms;
  }

  public static double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / Constants.kCountsPerRev;
    double wheelRotations = motorRotations / Constants.kSensorGearRatio;
    double positionMeters =
        wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches));
    return positionMeters;
  }
}
