package frc.robot.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorMap {
  default Command rumble() {
    return none();
  }

  Trigger levelOne();

  Trigger levelTwo();

  Trigger levelThree();

  Trigger levelFour();

  Trigger intake();

  Trigger outtake();

  Trigger modeSwitch();

  Trigger climberUp();

  Trigger climberDown();

  Trigger resetElevator();
}
