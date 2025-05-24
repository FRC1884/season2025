package frc.robot.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class XboxDriverMap extends CommandXboxController implements DriverMap {
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public XboxDriverMap(int port) {
    super(port);
  }

  @Override
  public Trigger leftAlign() {
    return leftBumper();
  }

  @Override
  public Trigger rightAlign() {
    return rightBumper();
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
    return () -> -getRightX();
  }

  @Override
  public Trigger resetOdometry() {
    return back();
  }

  @Override
  public Trigger stopWithX() {
    return x();
  }

  @Override
  public Trigger coralStation() {
    return new Trigger(() -> this.getRightTriggerAxis() > 0.5);
  }

  @Override
  public Trigger slowMode() {
    return new Trigger(() -> this.getLeftTriggerAxis() > 0.5);
  }

  @Override
  public Command rumble() {
    return startEnd(
        () -> getHID().setRumble(kBothRumble, 1), () -> getHID().setRumble(kBothRumble, 0));
  }
}
