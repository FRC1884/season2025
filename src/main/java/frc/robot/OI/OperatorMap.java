package frc.robot.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorMap {
  default Command rumble() {
    return none();
  }

  Trigger Idle();

  Trigger LevelOne();

  Trigger LevelTwo();

  Trigger LevelThree();

  Trigger LevelFour();

  Trigger Intake();

  Trigger Outake();

  Trigger WantsCoral();

  Trigger WantsAlgae();

  Trigger Testing();

  Trigger Source();

  Trigger StopIntake();

  Trigger ModeSwitch();
}
