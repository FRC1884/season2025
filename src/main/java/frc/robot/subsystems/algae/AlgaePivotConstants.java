package frc.robot.subsystems.algae;

import java.util.function.DoubleSupplier;

public final class AlgaePivotConstants {
  public static final int PIVOT_ID = 61; // TODO: find motor id
  public static final boolean INVERTED = true;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = false;
  public static final double FORWARD_LIMIT = 1.0, REVERSE_LIMIT = 0.0; // TODO: find limits

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final DoubleSupplier kP = () -> 5;
  public static final DoubleSupplier kI = () -> 0.0;
  public static final DoubleSupplier kD = () -> 0.0;
}
