package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }

  @Override
  public Trigger idle() {
    return button(4);
  }

  @Override
  public Trigger levelOne() {
    return button(7);
  }

  @Override
  public Trigger levelTwo() {
    return button(9);
  }

  @Override
  public Trigger levelThree() {
    return button(10);
  }

  @Override
  public Trigger levelFour() {
    return button(11);
  }

  @Override
  public Trigger intake() {
    return button(3);
  }

  @Override
  public Trigger outtake() {
    return button(2);
  }

  @Override
  public Trigger testing() {
    return button(6);
  }

  @Override
  public Trigger wantsCoral() {
    return button(8);
  }

  @Override
  public Trigger wantsAlgae() {
    return button(12);
  }

  @Override
  public Trigger source() {
    return button(5);
  }

  @Override
  public Trigger modeSwitch() {
    return button(1);
  }

  @Override
  public Trigger stopIntake() {
    return button(6);
  }

  @Override
  public Trigger deAlgaefyL3() {
    return button(8);
  }

  public Trigger deAlgaefyL2() {
    return button(12);
  }

  @Override
  public Trigger climberUp() {
    return axisGreaterThan(0, 0.5);
  }

  @Override
  public Trigger climberDown() {
    return axisLessThan(0, -0.5);
  }
}
