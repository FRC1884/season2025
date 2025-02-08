package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger TESTING() {
    return button(1);
  }

  @Override
  public Trigger level_one() {
    return button(2);
  }

  @Override
  public Trigger level_two() {
    return button(3);
  }

  @Override
  public Trigger level_three() {
    return button(4);
  }

  @Override
  public Trigger level_four() {
    return button(5);
  }

  @Override
  public Trigger source() {
    return button(6);
  }

  @Override
  public Trigger shallow() {
    return button(7);
  }

  @Override
  public Trigger deep() {
    return button(8);
  }

  @Override
  public Trigger intake() {
    return button(9);
  }

  @Override
  public Trigger outake() {
    return button(10);
  }
}
