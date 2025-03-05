package frc.robot.commands;

import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;
import static frc.robot.subsystems.Superstructure.SuperStates.INTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_FOUR;
import static frc.robot.subsystems.Superstructure.SuperStates.LF_FLICK;
import static frc.robot.subsystems.Superstructure.SuperStates.LF_OUTTAKE;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoCommands {
  public static void registerAutoCommands(Superstructure superstructure, SwerveSubsystem drive) {
    /** Write all the auto named commands here */
    /** Overriding commands */

    // overrides the x axis
    NamedCommands.registerCommand(
        "Override Coral Offset", DriveCommands.overridePathplannerCoralOffset(() -> 2.0));

    // clears all override commands in the x and y direction
    NamedCommands.registerCommand("Clear XY Override", DriveCommands.clearXYOverrides());

    /** Subsystem commands */
    NamedCommands.registerCommand("L4 Elevator", superstructure.setSuperStateCmd(LEVEL_FOUR));

    NamedCommands.registerCommand(
        "L4 Score",
        Commands.sequence(
            superstructure.setSuperStateCmd(LF_OUTTAKE),
            Commands.waitSeconds(0.38),
            superstructure.setSuperStateCmd(LF_FLICK),
            Commands.waitSeconds(0.2)));

    NamedCommands.registerCommand(
        "Source Align", DriveCommands.alignToNearestCoralStationCommand(drive, () -> 0));

    NamedCommands.registerCommand("Source", superstructure.setSuperStateCmd(INTAKE));

    NamedCommands.registerCommand("Stow", superstructure.setSuperStateCmd(IDLING));

    NamedCommands.registerCommand(
        "Left Align 1",
        Commands.deadline(
            Commands.waitSeconds(1), DriveCommands.alignToReefCommand(drive, () -> true, () -> 1)));

    NamedCommands.registerCommand(
        "Right Align 1",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommand(drive, () -> false, () -> 1)));

    NamedCommands.registerCommand(
        "Left Align 2",
        Commands.deadline(
            Commands.waitSeconds(1), DriveCommands.alignToReefCommand(drive, () -> true, () -> 2)));

    NamedCommands.registerCommand(
        "Right Align 2",
        Commands.deadline(
            Commands.waitSeconds(1.5),
            DriveCommands.alignToReefCommand(drive, () -> false, () -> 2)));

    NamedCommands.registerCommand(
        "Left Align 3",
        Commands.deadline(
            Commands.waitSeconds(1), DriveCommands.alignToReefCommand(drive, () -> true, () -> 3)));

    NamedCommands.registerCommand(
        "Right Align 3",
        Commands.deadline(
            Commands.waitSeconds(2.25),
            DriveCommands.alignToReefCommand(drive, () -> false, () -> 3)));

    NamedCommands.registerCommand(
        "Left Align 4",
        Commands.deadline(
            Commands.waitSeconds(1), DriveCommands.alignToReefCommand(drive, () -> true, () -> 4)));

    NamedCommands.registerCommand(
        "Right Align 4",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommand(drive, () -> false, () -> 4)));

    NamedCommands.registerCommand(
        "Left Align 5",
        Commands.deadline(
            Commands.waitSeconds(1), DriveCommands.alignToReefCommand(drive, () -> true, () -> 5)));

    NamedCommands.registerCommand(
        "Right Align 5",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommand(drive, () -> false, () -> 5)));

    NamedCommands.registerCommand(
        "Left Align 6",
        Commands.deadline(
            Commands.waitSeconds(1), DriveCommands.alignToReefCommand(drive, () -> true, () -> 6)));

    NamedCommands.registerCommand(
        "Right Align 6",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommand(drive, () -> false, () -> 6)));
  }
}
