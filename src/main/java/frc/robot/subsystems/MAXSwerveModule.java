// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SparkMaxPIDController drivePIDController;
  private final SparkMaxPIDController turnPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    driveSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turnSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveEncoder = driveSparkMax.getEncoder();
    turnEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivePIDController = driveSparkMax.getPIDController();
    turnPIDController = turnSparkMax.getPIDController();
    drivePIDController.setFeedbackDevice(driveEncoder);
    turnPIDController.setFeedbackDevice(turnEncoder);

    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_METERS_POSITION_FACTOR);
    driveEncoder.setVelocityConversionFactor(
        ModuleConstants.DRIVE_ENCODER_METERS_PER_SECOND_VELOCITY_FACTOR);

    turnEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_RADIANS_POSITION_FACTOR);
    turnEncoder.setVelocityConversionFactor(
        ModuleConstants.TURN_ENCODER_RADIANS_PER_SECOND_VELOCITY_FACTOR);

    turnEncoder.setInverted(ModuleConstants.turnEncoderInverted);

    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(
        ModuleConstants.TURN_ENCODER_POSITION_PID_MIN_INPUT_RADIANS);
    turnPIDController.setPositionPIDWrappingMaxInput(
        ModuleConstants.TURN_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);

    drivePIDController.setP(ModuleConstants.DRIVE_KP);
    drivePIDController.setI(ModuleConstants.DRIVE_KI);
    drivePIDController.setD(ModuleConstants.DRIVE_KD);
    drivePIDController.setFF(ModuleConstants.DRIVE_FF);
    drivePIDController.setOutputRange(
        ModuleConstants.DRIVE_MIN_OUTPUT, ModuleConstants.DRIVE_MAX_OUTPUT);

    turnPIDController.setP(ModuleConstants.TURN_KP);
    turnPIDController.setI(ModuleConstants.TURN_KI);
    turnPIDController.setD(ModuleConstants.TURN_KD);
    turnPIDController.setFF(ModuleConstants.TURN_FF);
    turnPIDController.setOutputRange(
        ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);

    driveSparkMax.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
    turnSparkMax.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE);
    driveSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT_AMPS);
    turnSparkMax.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT_AMPS);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(turnEncoder.getPosition()));

    drivePIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    turnSparkMax.setVoltage(0.0);
    driveSparkMax.setVoltage(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return driveEncoder.getVelocity();
  }
}
