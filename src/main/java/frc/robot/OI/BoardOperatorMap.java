package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.conduit.schema.Joystick;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger shoot() {
    return button(5);
  }

  @Override
  public Trigger level_one() {
    return povUp();
  }

  @Override
  public Trigger level_two() {
    return povRight();
  }

  @Override
  public Trigger level_three() {
    return povLeft();
  }

  @Override
  public Trigger level_four() {
    return povLeft();
  }

  @Override
  public Trigger source() {
    return button(1);
  }

  @Override
  public Trigger shallow() {
    return button(2);
  }

  @Override
  public Trigger deep() {
    return button(3);
  }

  @Override
  public Trigger prepShot() {
    return button(7);
  }

  // TODO: add intake button
  @Override
  public Trigger intake() {
    return button(8);
  }
}
