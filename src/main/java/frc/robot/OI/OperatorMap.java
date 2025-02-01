package frc.robot.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorMap {
  Trigger shoot();

  Trigger level_one();

  Trigger level_two();

  Trigger level_three();

  Trigger level_four();

  Trigger source();

  Trigger shallow();

  Trigger deep();

  default Command rumble() {
    return none();
  }

  Trigger prepShot();

  Trigger intake();
}
