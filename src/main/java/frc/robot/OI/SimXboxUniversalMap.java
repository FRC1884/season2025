package frc.robot.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class SimXboxUniversalMap extends CommandXboxController implements DriverMap, OperatorMap {
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public SimXboxUniversalMap(int port) {
    super(port);
  }

  @Override
  public Trigger leftAlign() {
    return button(7);
  }

  @Override
  public Trigger rightAlign() {
    return button(8);
  }

  @Override
  public DoubleSupplier getXAxis() {
    return () -> -getLeftX();
  }

  @Override
  public DoubleSupplier getYAxis() {
    return () -> -getLeftY();
  }

  @Override
  public DoubleSupplier getRotAxis() {
    return () -> -getRawAxis(2);
  }

  @Override
  public Trigger pathToAmp() {
    return button(14);
  }

  @Override
  public Trigger resetOdometry() {
    return button(11);
  }

  @Override
  public Trigger stopWithX() {
    return button(4);
  }

  @Override
  public Trigger testButton() {
    return button(1);
  }

  // would we need to mutex this through a subsys req if we switch to the maple swerve skeleton?
  @Override
  public Command rumble() {
    return startEnd(
        () -> getHID().setRumble(kBothRumble, 1), () -> getHID().setRumble(kBothRumble, 0));
  }

  @Override
  public Trigger coralStation() {
    return button(2);
  }

  @Override
  public Trigger slowMode() {
    return button(3);
  }

  @Override
  public Trigger idle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Idle'");
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
  public Trigger testing() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Testing'");
  }

  @Override
  public Trigger wantsCoral() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'WantsCoral'");
  }

  @Override
  public Trigger wantsAlgae() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'WantsAlgae'");
  }

  @Override
  public Trigger source() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Source'");
  }

  @Override
  public Trigger modeSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ModeSwitch'");
  }

  @Override
  public Trigger stopIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'StopIntake'");
  }

  @Override
  public Trigger deAlgaefyL2() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'DeAlgaefyL2'");
  }

  @Override
  public Trigger deAlgaefyL3() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'DeAlgaefyL3'");
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
}
