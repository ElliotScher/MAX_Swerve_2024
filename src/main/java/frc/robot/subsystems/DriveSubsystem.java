// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(
          DriveConstants.FRONT_LEFT_DRIVE_CAN_ID,
          DriveConstants.FRONT_LEFT_TURN_CAN_ID,
          DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVE_CAN_ID,
          DriveConstants.FRONT_RIGHT_TURN_CAN_ID,
          DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule rearLeft =
      new MAXSwerveModule(
          DriveConstants.REAR_LEFT_DRIVE_CAN_ID,
          DriveConstants.REAR_LEFT_TURN_CAN_ID,
          DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule rearRight =
      new MAXSwerveModule(
          DriveConstants.REAR_RIGHT_DRIVE_CAN_ID,
          DriveConstants.REAR_RIGHT_TURN_CAN_ID,
          DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule[] modules = {
    frontLeft, frontRight, rearLeft, rearRight
  }; // FL, FR, BL, BR

  private final AHRS gyro = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDirection = 0.0;
  private double currentTranslationMagnitude = 0.0;

  private SlewRateLimiter magnitudeSlewRateLimiter =
      new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter rotationSlewRateLimiter =
      new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
  private double previousTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.DRIVE_KINEMATICS,
          Rotation2d.fromDegrees(gyro.getAngle()),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        () -> DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()),
        this::driveFieldRelative,
        new HolonomicPathFollowerConfig(
            DriveConstants.MAX_LINEAR_SPEED,
            DriveConstants.DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        this);
  }

  @Override
  public void periodic() {
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMagnitude != 0.0) {
        directionSlewRate =
            Math.abs(DriveConstants.DIRECTION_SLEW_RATE / currentTranslationMagnitude);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - previousTime;
      double angleDif =
          SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDirection);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDirection =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDirection, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = magnitudeSlewRateLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMagnitude
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMagnitude = magnitudeSlewRateLimiter.calculate(0.0);
        } else {
          currentTranslationDirection =
              SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI);
          currentTranslationMagnitude = magnitudeSlewRateLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDirection =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDirection, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = magnitudeSlewRateLimiter.calculate(0.0);
      }
      previousTime = currentTime;

      xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
      ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
      currentRotation = rotationSlewRateLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_LINEAR_SPEED;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_LINEAR_SPEED;
    double rotDelivered = currentRotation * DriveConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_LINEAR_SPEED);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Drives the robot field relative based on ChassisSpeeds object */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  /** Drives the robot robot relative based on ChassisSpeeds object */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_LINEAR_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_IS_INVERTED ? -1.0 : 1.0);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }
}
