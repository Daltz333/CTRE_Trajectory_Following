// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.io.IOException;
import java.nio.file.Path;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private Drivetrain drivetrain = new Drivetrain();

  private Trajectory trajectory;

  // for following our trajectory
  private final RamseteController ramseteController = new RamseteController();

  // timer to step through the trajectory
  private Timer timer;

  @Override
  public void robotInit() {
    String trajectoryJson = "Unnamed.wpilib.json";

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory", false);
    }

    drivetrain.plotTrajectory(trajectory);
  }

  @Override
  public void robotPeriodic() {
    drivetrain.periodic();
  }

  @Override
  public void autonomousInit() {
    timer = new Timer();
    timer.start();

    drivetrain.resetOdometry(trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    if (timer.get() < trajectory.getTotalTimeSeconds()) {
      // Get the desired pose from the trajectory.
      var desiredPose = trajectory.sample(timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = ramseteController.calculate(drivetrain.getPose(), desiredPose);

      // Set the linear and angular speeds.
      drivetrain.drive(refChassisSpeeds.vxMetersPerSecond, -refChassisSpeeds.omegaRadiansPerSecond);
    } else {
      drivetrain.drive(0, 0);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    drivetrain.drive();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    System.out.println("Running in Sim");
    drivetrain.simulationPeriodic();
  }
}
