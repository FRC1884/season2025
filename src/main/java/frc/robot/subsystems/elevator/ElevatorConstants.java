package frc.robot.subsystems.elevator;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class ElevatorConstants {
  public static final int LEFT_ELEVATOR = 31; // TODO: Change to correct Motor ID's
  public static final boolean LEFT_INVERTED = true;
  public static final int RIGHT_ELEVATOR = 32; // TODO: Change to correct Motor ID's
  public static final boolean RIGHT_INVERTED = true;
  public static final int DIOPort = 0;

  public static final boolean isFlex = true; // if the motors on the Elevator are flex motors

  public static final DoubleSupplier kPUp =
      new LoggedTunableNumber("Elevator/kPUp", 0.14); // 0.4 pid for neo
  public static final DoubleSupplier kIUp = new LoggedTunableNumber("Elevator/kIUp", 0.0000005);
  public static final DoubleSupplier kDUp = new LoggedTunableNumber("Elevator/kDUp", 8.0);
  public static final DoubleSupplier kPDown =
      new LoggedTunableNumber("Elevator/kPDown", 0.075); // 0.125
  public static final DoubleSupplier kIDown =
      new LoggedTunableNumber("Elevator/kIDown", 0.0); // 0.000001
  public static final DoubleSupplier kDDown = new LoggedTunableNumber("Elevator/kDDown", 20); // 10
}
