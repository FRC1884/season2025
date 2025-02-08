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
  public Trigger alignToGamePiece() {
    return button(7);
  }

  @Override
  public Trigger alignToSpeaker() {
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
  public Trigger intake() {
    return button(10);
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
  public Trigger TESTING() {
    return null;
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
  public Trigger level_one() {
    return button(9);
  }

  @Override
  public Trigger level_two() {
    return button(10);
  }

  @Override
  public Trigger level_three() {
    return button(11);
  }

  @Override
  public Trigger level_four() {
    return button(12);
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
    return button(13);
  }

  @Override
  public Trigger outake() {
    return button(14);
  }
}
