package frc.robot.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface DriverMap {
  Trigger alignToGamePiece();

  Trigger alignToSpeaker();

  DoubleSupplier getXAxis();

  DoubleSupplier getYAxis();

  DoubleSupplier getRotAxis();

  Trigger pathToAmp();

  Trigger resetOdometry();

  Trigger stopWithX();

  Trigger testButton();

  Trigger coralStation();

  Trigger slowMode();

  default Command rumble() {
    return none();
  }
}
