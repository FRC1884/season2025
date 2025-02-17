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
  public Trigger Idle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Idle'");
  }

  @Override
  public Trigger LevelOne() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelOne'");
  }

  @Override
  public Trigger LevelTwo() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelTwo'");
  }

  @Override
  public Trigger LevelThree() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelThree'");
  }

  @Override
  public Trigger LevelFour() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LevelFour'");
  }

  @Override
  public Trigger Intake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Intake'");
  }

  @Override
  public Trigger Outake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Outake'");
  }

  @Override
  public Trigger Testing() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Testing'");
  }

  @Override
  public Trigger WantsCoral() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'WantsCoral'");
  }

  @Override
  public Trigger WantsAlgae() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'WantsAlgae'");
  }

  @Override
  public Trigger Source() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Source'");
  }

  @Override
  public Trigger ModeSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ModeSwitch'");
  }

  @Override
  public Trigger StopIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'StopIntake'");
  }
}
