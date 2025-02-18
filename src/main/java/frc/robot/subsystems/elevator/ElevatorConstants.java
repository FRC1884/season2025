package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

public final class ElevatorConstants {
  public static final int LEFT_ELEVATOR = 31; // TODO: Change to correct Motor ID's
  public static final boolean LEFT_INVERTED = true;
  public static final int RIGHT_ELEVATOR = 32; // TODO: Change to correct Motor ID's
  public static final boolean RIGHT_INVERTED = true;
  public static final int DIOPort = 0;

  public static final boolean isFlex = true; // if the motors on the Elevator are flex motors

  public static final DoubleSupplier kPUp = () -> 0.14; // 0.4 pid for neo
  public static final DoubleSupplier kIUp = () -> 0.0000005;
  public static final DoubleSupplier kDUp = () -> 8.0;
  public static final DoubleSupplier kPDown = () -> 0.018;
  public static final DoubleSupplier kIDown = () -> 0.0;
  public static final DoubleSupplier kDDown = () -> 3;
}
