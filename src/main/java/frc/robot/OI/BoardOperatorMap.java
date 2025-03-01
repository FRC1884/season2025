package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger levelOne() {
    return button(1);
  }

  @Override
  public Trigger levelTwo() {
    return button(11);
  }

  @Override
  public Trigger levelThree() {
    return button(12);
  }

  @Override
  public Trigger levelFour() {
    return button(10);
  }

  @Override
  public Trigger intake() {
    return button(2);
  }

  @Override
  public Trigger outtake() {
    return button(3);
  }

  @Override
  public Trigger modeSwitch() {
    return button(9);
  }

  @Override
  public Trigger climberUp() {
    return axisGreaterThan(1, 0.5);
  }

  @Override
  public Trigger climberDown() {
    return axisLessThan(1, -0.5);
  }
}
