package frc.robot.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;

public interface OperatorMap {
  default Command rumble() {
    return none();
  }
}
