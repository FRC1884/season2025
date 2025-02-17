package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

public final class ElevatorConstants {
  public static final int LEFT_ELEVATOR = 42; // TODO: Change to correct Motor ID's
  public static final boolean LEFT_INVERTED = true;
  public static final int RIGHT_ELEVATOR = 41; // TODO: Change to correct Motor ID's
  public static final boolean RIGHT_INVERTED = true;
  public static final double PULLEY_RADIUS = 0.016;
  public static final double restingRot = -0.8;
  public static final int DIOPort = 1;

  public static final boolean isFlex = true; // if the motors on the Elevator are flex motors

  public static final DoubleSupplier kP = () -> 0.1; // 0.4 pid for neo
  public static final DoubleSupplier kI = () -> 0.0;
  public static final DoubleSupplier kD = () -> 1;
}
