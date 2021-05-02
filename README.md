# CTRE Trajectory Following

This is an example project that I wrote that adapts and modifies the [WPILib RamseteController example](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecontroller), to use a vendor motor contorller (CTRE TalonSRX). This example uses the KoP chassis base as a substitution for characterization constants, shouldn't be too hard to actually get the constants for it.

## Important Learning Points

**Units! Units! Units!**

Units are everything. My biggest advise is to use meters for everything. Robot measurements? Meters! Even the slightest unit mismatch is absolutely atrocious to debug.

**Don't use CTRE Physics**

It's confusing, and doesn't integrate well with all the WPILib sim work. Not quite sure why they did it.

**Here's some CTRE Native Unit Conversion Methods**

```java
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
```
using the above, it makes it very easy to convert between actual units, and CTRE's units.

**Tuning Constants**

- Robot overshoots its turns but overall follows well.
  - Try tinkering with your PID values.
- It vaguely follows it, but overall quite bad.
  - Verify your values are all correct, then try tinkering with your feedforward
- See [this troubleshooting article](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/troubleshooting.html).

**PathWeaver**

- Overall great tool. Always export it in meters.
- Use WPILib 2021.3.1 or greater. There was a bug in previous versions that caused units with inputs of inches to not export right.

**Learn how to graph your inputs and outputs**

- Comparing expected voltage vs wanted voltage (Useful for verifying PID sometimes)
- Visualizing the generated path, to verify that it's possible for the robot to achieve it
- Comparing actual wheel speeds vs wanted wheel speeds (also useful for PID)

# Great Stuff!

Overall, this was absolutely a blast to work on. I'm excited to tinker with some of the new stuff upcoming for WPILib (hint hint mechanism2d).
