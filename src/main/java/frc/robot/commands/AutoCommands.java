package frc.robot.commands;

import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;
import static frc.robot.subsystems.Superstructure.SuperStates.INTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_FOUR;
import static frc.robot.subsystems.Superstructure.SuperStates.OUTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.SOURCE;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;

public class AutoCommands {
  public static void registerAutoCommands(Superstructure superstructure) {
    /** Write all the auto named commands here */
    /** Overriding commands */

    // overrides the x axis
    NamedCommands.registerCommand(
        "Override Coral Offset", DriveCommands.overridePathplannerCoralOffset(() -> 2.0));

    // clears all override commands in the x and y direction
    NamedCommands.registerCommand("Clear XY Override", DriveCommands.clearXYOverrides());

    /** Subsystem commands */
    NamedCommands.registerCommand(
        "L4 Score",
        Commands.sequence(
            superstructure.setSuperStateCmd(LEVEL_FOUR),
            Commands.waitSeconds(0.5),
            superstructure.setSuperStateCmd(OUTAKE),
            Commands.waitUntil(superstructure.hasGamePiece().negate()),
            superstructure.setSuperStateCmd(IDLING),
            Commands.waitSeconds(0.5)));

    NamedCommands.registerCommand(
        "Coral Station Intake Elevator", superstructure.setSuperStateCmd(SOURCE));

    NamedCommands.registerCommand("Intake", superstructure.setSuperStateCmd(INTAKE));

    NamedCommands.registerCommand("Stow", superstructure.setSuperStateCmd(IDLING));
  }
}
