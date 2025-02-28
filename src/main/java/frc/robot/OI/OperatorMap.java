package frc.robot.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorMap {
  default Command rumble() {
    return none();
  }

  Trigger idle();

  Trigger levelOne();

  Trigger levelTwo();

  Trigger levelThree();

  Trigger levelFour();

  Trigger intake();

  Trigger outtake();

  Trigger wantsCoral();

  Trigger wantsAlgae();

  Trigger testing();

  Trigger source();

  Trigger stopIntake();

  Trigger deAlgaefyL2();

  Trigger deAlgaefyL3();

  Trigger modeSwitch();

  Trigger climberUp();

  Trigger climberDown();
}
