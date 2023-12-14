// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_LINEAR_SPEED = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
    public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot

    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVE_CAN_ID = 13;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 15;
    public static final int REAR_RIGHT_DRIVE_CAN_ID = 17;

    public static final int FRONT_LEFT_TURN_CAN_ID = 10;
    public static final int REAR_LEFT_TURN_CAN_ID = 12;
    public static final int FRONT_RIGHT_TURN_CAN_ID = 14;
    public static final int REAR_RIGHT_TURN_CAN_ID = 16;

    public static final boolean GYRO_IS_INVERTED = false;
  }

  public static final class ModuleConstants {
    public static final int DRIVE_PINION_TEETH = 14;
    public static final boolean turnEncoderInverted = true;

    public static final double DRIVE_MOTOR_FREE_SPEED_RPS =
        NeoMotorConstants.NEO_FREE_SPEED_RPM / 60.0;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
        (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVE_MOTOR_REDUCTION;

    public static final double DRIVE_ENCODER_METERS_POSITION_FACTOR =
        (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_MOTOR_REDUCTION;
    public static final double DRIVE_ENCODER_METERS_PER_SECOND_VELOCITY_FACTOR =
        ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0;

    public static final double TURN_ENCODER_RADIANS_POSITION_FACTOR = (2 * Math.PI);
    public static final double TURN_ENCODER_RADIANS_PER_SECOND_VELOCITY_FACTOR =
        (2 * Math.PI) / 60.0;

    public static final double TURN_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0;
    public static final double TURN_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
        TURN_ENCODER_RADIANS_POSITION_FACTOR;

    public static final double DRIVE_KP = 0.04;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_FF =
        1 / DRIVE_WHEEL_FREE_SPEED_RPS; // replace with simplemotorfeedforward value with gains from
    // characterization
    public static final double DRIVE_MIN_OUTPUT = -1;
    public static final double DRIVE_MAX_OUTPUT = 1;

    public static final double TURN_KP = 1;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0;
    public static final double TURN_FF = 0;
    public static final double TURN_MIN_OUTPUT = -1;
    public static final double TURN_MAX_OUTPUT = 1;

    public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT_AMPS = 50;
    public static final int TURN_MOTOR_CURRENT_LIMIT_AMPS = 20;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DRIVE_DEADBAND = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double NEO_FREE_SPEED_RPM = 5676;
  }
}
