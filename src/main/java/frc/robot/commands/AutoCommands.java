package frc.robot.commands;

import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;
import static frc.robot.subsystems.Superstructure.SuperStates.INTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_FOUR;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_THREE;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_TWO;
import static frc.robot.subsystems.Superstructure.SuperStates.LF_FLICK;
import static frc.robot.subsystems.Superstructure.SuperStates.LF_OUTTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.OUTTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.STOP_INTAKE;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoCommands {
  public static void registerAutoCommands(Superstructure superstructure, SwerveSubsystem drive) {
    /** Write all the auto named commands here */

    /** Subsystem commands */
    NamedCommands.registerCommand("L4 Elevator", superstructure.setSuperStateCmd(LEVEL_FOUR));

    NamedCommands.registerCommand(
        "L4 Score",
        Commands.sequence(
            superstructure.setSuperStateCmd(LF_OUTTAKE),
            Commands.waitSeconds(0.38),
            superstructure.setSuperStateCmd(LF_FLICK),
            Commands.waitSeconds(0.2)));
    NamedCommands.registerCommand("Net Score", superstructure.setSuperStateCmd(OUTTAKE));

    NamedCommands.registerCommand(
        "L3 Algae Pickup",
        Commands.sequence(
            superstructure.setSuperStateCmd(LEVEL_THREE),
            Commands.waitSeconds(0.4),
            superstructure.setSuperStateCmd(INTAKE),
            Commands.waitUntil(superstructure.hasGamePiece()),
            superstructure.setSuperStateCmd(STOP_INTAKE)));

    NamedCommands.registerCommand(
        "Source",
        Commands.sequence(
            superstructure.setSuperStateCmd(IDLING),
            Commands.waitSeconds(0.1),
            superstructure.setSuperStateCmd(INTAKE)));

    NamedCommands.registerCommand(
        "L2 Algae Pickup",
        Commands.sequence(
            superstructure.setSuperStateCmd(LEVEL_TWO),
            Commands.waitSeconds(0.4),
            superstructure.setSuperStateCmd(INTAKE),
            Commands.waitSeconds(0.2),
            superstructure.setSuperStateCmd(STOP_INTAKE)));
    NamedCommands.registerCommand("Stow", superstructure.setSuperStateCmd(IDLING));

    NamedCommands.registerCommand(
        "Source Align",
        Commands.deadline(
            Commands.waitSeconds(2), DriveCommands.alignToNearestCoralStationCommandAuto(drive)));

    NamedCommands.registerCommand(
        "Left Align 1",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> true, () -> 1)));

    NamedCommands.registerCommand(
        "Right Align 1",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> false, () -> 1)));

    NamedCommands.registerCommand(
        "Left Align 2",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> true, () -> 2)));

    NamedCommands.registerCommand(
        "Right Align 2",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> false, () -> 2)));

    NamedCommands.registerCommand(
        "Left Align 3",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> true, () -> 3)));

    NamedCommands.registerCommand(
        "Right Align 3",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> false, () -> 3)));

    NamedCommands.registerCommand(
        "Left Align 4",
        Commands.deadline(
            Commands.waitSeconds(4),
            DriveCommands.alignToReefCommandAuto(drive, () -> true, () -> 4)));

    NamedCommands.registerCommand(
        "Right Align 4",
        Commands.deadline(
            Commands.waitSeconds(4),
            DriveCommands.alignToReefCommandAuto(drive, () -> false, () -> 4)));

    NamedCommands.registerCommand(
        "Left Align 5",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> true, () -> 5)));

    NamedCommands.registerCommand(
        "Right Align 5",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> false, () -> 5)));

    NamedCommands.registerCommand(
        "Left Align 6",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> true, () -> 6)));

    NamedCommands.registerCommand(
        "Right Align 6",
        Commands.deadline(
            Commands.waitSeconds(1),
            DriveCommands.alignToReefCommandAuto(drive, () -> false, () -> 6)));
  }
}
