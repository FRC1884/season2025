package frc.robot.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

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
    return startEnd(
        () -> getHID().setRumble(kBothRumble, 1), () -> getHID().setRumble(kBothRumble, 0));
  }

  @Override
  public Trigger levelOne() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelOne'");
  }

  @Override
  public Trigger levelTwo() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelTwo'");
  }

  @Override
  public Trigger levelThree() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelThree'");
  }

  @Override
  public Trigger levelFour() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelFour'");
  }

  @Override
  public Trigger intake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Intake'");
  }

  @Override
  public Trigger outtake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Outake'");
  }

  @Override
  public Trigger modeSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ModeSwitch'");
  }

  @Override
  public Trigger climberUp() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ClimberUp'");
  }

  @Override
  public Trigger climberDown() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ClimberDown'");
  }

  @Override
  public Trigger resetElevator() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetElevator'");
  }
}
