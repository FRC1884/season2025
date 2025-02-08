package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
  public static final int LEFT_ELEVATOR = 24; // TODO: Change to correct Motor ID's
  public static final int RIGHT_ELEVATOR = 23; // TODO: Change to correct Motor ID's
  public static final double PULLEY_RADIUS = 0.016;
  public static final double startingPoint =
      0.0; // change for starting meters so that we know how much we are at

  public static final boolean isFlex = true; // if the motors on the Elevator are flex motors

  public static final double kP = 0.02; // 0.02 for Vortex | 0.4 pid for neo
  public static final double kI = 0.0;
  public static final double kD = 0.0;
}
