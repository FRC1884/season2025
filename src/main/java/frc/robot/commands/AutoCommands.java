package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

public class AutoCommands {
  public static void registerAutoCommands() {
    /** Write all the auto named commands here */
    /** Overriding commands */

    // overrides the x axis
    NamedCommands.registerCommand(
        "Override Coral Offset", DriveCommands.overridePathplannerCoralOffset(() -> 2.0));

    // clears all override commands in the x and y direction
    NamedCommands.registerCommand("Clear XY Override", DriveCommands.clearXYOverrides());

    /** Subsystem commands */
  }
}
