package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOperatorMap extends CommandXboxController implements OperatorMap {

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public XboxOperatorMap(int port) {
    super(port);
  }

  @Override
  public Command rumble() {
    return OperatorMap.super.rumble();
  }

  @Override
  public Trigger TESTING() {
    return null;
  }

  @Override
  public Trigger level_one() {
    return button(1);
  }

  @Override
  public Trigger level_two() {
    return button(2);
  }

  @Override
  public Trigger level_three() {
    return button(3);
  }

  @Override
  public Trigger level_four() {
    return button(4);
  }

  @Override
  public Trigger source() {
    return button(5);
  }

  @Override
  public Trigger shallow() {
    return button(6);
  }

  @Override
  public Trigger deep() {
    return button(7);
  }

  @Override
  public Trigger intake() {
    return button(8);
  }

  @Override
  public Trigger outake() {
    return button(9);
  }
}
