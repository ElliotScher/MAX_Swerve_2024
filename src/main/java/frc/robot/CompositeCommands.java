package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public final class CompositeCommands {
  public static final Command drive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      boolean fieldRelative,
      boolean rateLimit) {
    return drive.run(
        () ->
            drive.drive(
                -MathUtil.applyDeadband(xSupplier.getAsDouble(), OIConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(ySupplier.getAsDouble(), OIConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(omegaSupplier.getAsDouble(), OIConstants.DRIVE_DEADBAND),
                fieldRelative,
                rateLimit));
  }

  public static final Command setX(DriveSubsystem drive) {
    return drive.runOnce(drive::setX);
  }
}
