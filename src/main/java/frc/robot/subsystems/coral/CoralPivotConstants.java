package frc.robot.subsystems.coral;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class CoralPivotConstants {
  public static final int PIVOT_ID = 51;
  public static final boolean INVERTED = true;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = false;
  public static final double FORWARD_LIMIT = 0.5, REVERSE_LIMIT = 0.05;

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final DoubleSupplier kP = new LoggedTunableNumber("CoralPivot/kP", 4.5);
}
