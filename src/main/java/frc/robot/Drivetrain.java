// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;

public class Drivetrain {
  // create our motor controller objects, ensure they are the WPI variant for DriveSim compatibility
  private WPI_TalonSRX motorLeftMaster = new WPI_TalonSRX(Constants.kMotorLeftMaster);
  private WPI_TalonSRX motorLeftFollower = new WPI_TalonSRX(Constants.kMotorLeftFollower);
  private WPI_TalonSRX motorRightMaster = new WPI_TalonSRX(Constants.kMotorRightMaster);
  private WPI_TalonSRX motorRightFollower = new WPI_TalonSRX(Constants.kMotorRightFollower);

  private AHRS gyro = new AHRS(Port.kMXP);

  private XboxController driver = new XboxController(0);

  // Simulation model of the drivetrain
  private DifferentialDrivetrainSim driveSim =
      new DifferentialDrivetrainSim(
          DCMotor.getCIM(2), // 2 CIMS on each side of the drivetrain.
          Constants.kGearRatio, // Standard AndyMark Gearing reduction.
          2.1, // MOI of 2.1 kg m^2 (from CAD model).
          26.5, // Mass of the robot is 26.5 kg.
          Units.inchesToMeters(Constants.kWheelRadiusInches),
          Constants.kTrackWidthMeters, // Distance between wheels in meters
          null);

  // Object for simulated inputs into Talon.
  private TalonSRXSimCollection leftDriveSim = motorLeftMaster.getSimCollection();
  private TalonSRXSimCollection rightDriveSim = motorRightMaster.getSimCollection();

  // create field object for visualizations
  private Field2d field = new Field2d();

  // create robot odometry, "where" our robot is
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  // left and right pid controller for wheel speeds
  private final PIDController leftPIDController = new PIDController(1, 0, 0);
  private final PIDController rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.kTrackWidthMeters);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

  SimDouble gyroSim =
      new SimDouble(
          SimDeviceDataJNI.getSimValueHandle(
              SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

  public Drivetrain() {
    // reset to default to ensure that no sensitive settings has been messed with
    motorLeftMaster.configFactoryDefault();
    motorRightMaster.configFactoryDefault();

    motorLeftFollower.follow(motorLeftMaster);
    motorRightFollower.follow(motorRightMaster);

    // configure open loop ramp rate for smoother acceleration
    motorLeftMaster.configOpenloopRamp(Constants.kDriveOpenLoopRampRate);
    motorRightMaster.configOpenloopRamp(Constants.kDriveOpenLoopRampRate);

    motorLeftFollower.setInverted(InvertType.FollowMaster);
    motorRightFollower.setInverted(InvertType.FollowMaster);

    // ensure left side is not inverted
    motorLeftMaster.setInverted(InvertType.None);
    motorLeftMaster.setSensorPhase(false);

    // reset our encoders
    motorLeftMaster.setSelectedSensorPosition(0);
    motorRightMaster.setSelectedSensorPosition(0);

    gyro.reset();

    Shuffleboard.getTab("Simulation").add(field);

    // invert right side only on real robot, as simulation expects both side positive
    if (RobotBase.isReal()) {
      motorRightMaster.setInverted(InvertType.InvertMotorOutput);
      motorRightMaster.setSensorPhase(true);

    } else {
      motorRightMaster.setInverted(InvertType.None);
      motorRightMaster.setSensorPhase(false);
    }
  }

  // update our drivetrain location
  public void periodic() {
    odometry.update(
        gyro.getRotation2d(),
        CTREUtil.nativeUnitsToDistanceMeters(motorLeftMaster.getSelectedSensorPosition()),
        CTREUtil.nativeUnitsToDistanceMeters(motorRightMaster.getSelectedSensorPosition()));

    field.setRobotPose(odometry.getPoseMeters());

    // debug information
    SmartDashboard.putNumber("Left Master Voltage", motorLeftMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Master Voltage", motorRightMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("Left Follower Voltage", motorLeftFollower.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Follower Voltage", motorRightFollower.getMotorOutputVoltage());

    SmartDashboard.putNumber(
        "Left Master Velocity",
        CTREUtil.velocityToNativeUnits(motorLeftMaster.getSelectedSensorVelocity()));

    SmartDashboard.putNumber(
        "Right Master Velocity",
        CTREUtil.velocityToNativeUnits(motorRightMaster.getSelectedSensorVelocity()));
  }

  // periodically called during simulation
  public void simulationPeriodic() {
    driveSim.setInputs(
        motorLeftMaster.getMotorOutputVoltage(), motorRightMaster.getMotorOutputVoltage());

    // update our simulation every 20ms
    driveSim.update(0.02);

    // update all of our sensors
    leftDriveSim.setQuadratureRawPosition(
        CTREUtil.distanceToNativeUnits(driveSim.getLeftPositionMeters()));
    leftDriveSim.setQuadratureVelocity(
        CTREUtil.velocityToNativeUnits(driveSim.getLeftVelocityMetersPerSecond()));
    rightDriveSim.setQuadratureRawPosition(
        CTREUtil.distanceToNativeUnits(driveSim.getRightPositionMeters()));
    rightDriveSim.setQuadratureVelocity(
        CTREUtil.velocityToNativeUnits(driveSim.getRightVelocityMetersPerSecond()));

    leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    gyroSim.set(-driveSim.getHeading().getDegrees());
  }

  public void resetOdometry(Pose2d pose) {
    motorLeftMaster.setSelectedSensorPosition(0);
    motorRightMaster.setSelectedSensorPosition(0);
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void plotTrajectory(Trajectory trajectory) {
      ArrayList<Pose2d> poses = new ArrayList<>();

      for (Trajectory.State pose : trajectory.getStates()) {
        poses.add(pose.poseMeters);
      }

      field.getObject("foo").setPoses(poses);
  }

  // TODO investigate if the math here is right
  public double getLeftVelocityMeters() {
    return Units.inchesToMeters(
        motorLeftMaster.getSelectedSensorVelocity() / Constants.kMagMultiplier);
  }

  public double getRightVelocityMeters() {
    return Units.inchesToMeters(
        motorRightMaster.getSelectedSensorVelocity() / Constants.kMagMultiplier);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        leftPIDController.calculate(getLeftVelocityMeters(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(getRightVelocityMeters(), speeds.rightMetersPerSecond);
    motorLeftMaster.setVoltage((leftOutput + leftFeedforward));
    motorRightMaster.setVoltage((rightOutput + rightFeedforward));
  }

  // normal teleop drive function
  public void drive() {
    double throttle = -driver.getY(Hand.kLeft); // invert our y due to joystick convention
    double turn = driver.getX(Hand.kRight);

    SmartDashboard.putNumber("Controller Throttle", throttle);
    SmartDashboard.putNumber("Controller Turn", turn);

    motorLeftMaster.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, turn);
    motorRightMaster.set(
        ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, -turn);
  }

  // trajectory following drive function
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
}
