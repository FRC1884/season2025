package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoCommands {
  public static void registerAutoCommands(Superstructure superstructure, SwerveSubsystem drive) {
    /** Write all the auto named commands here */
    NamedCommands.registerCommand(
        "Align",
        Commands.deadline(
            Commands.waitSeconds(2), DriveCommands.alignToNearestCoralStationCommandAuto(drive)));
  }
}
