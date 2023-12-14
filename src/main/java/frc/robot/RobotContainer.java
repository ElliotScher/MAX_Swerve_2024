// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.FeedForwardCharacterization;

public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();

  private final CommandXboxController driver =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand(
        "Example Command",
        Commands.print("This is an example command inside an autonomous routine!!!!"));

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        CompositeCommands.drive(
            drive,
            () -> driver.getLeftX(),
            () -> driver.getLeftY(),
            () -> driver.getRightX(),
            true,
            true));

    driver.x().whileTrue(CompositeCommands.setX(drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
