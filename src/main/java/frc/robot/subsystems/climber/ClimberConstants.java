package frc.robot.subsystems.climber;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class ClimberConstants {
  public static final int LEFT_CLIMBER = 23; // TODO: Change to correct Motor ID's
  public static final boolean LEFT_INVERTED = true;
  public static final int RIGHT_CLIMBER = 24; // TODO: Change to correct Motor ID's
  public static final boolean RIGHT_INVERTED = true;
  public static final double PULLEY_RADIUS = 0.016;
  public static final int DIOPort = 2;

  public static final boolean isFlex = false; // if the motors on the climber are flex motors

  public static final DoubleSupplier kP =
      new LoggedTunableNumber("Climber/kP", 0.4); // 0.4 pid for neo
  public static final DoubleSupplier kI = new LoggedTunableNumber("Climber/kI", 0);
  public static final DoubleSupplier kD = new LoggedTunableNumber("Climber/kD", 0);
}
