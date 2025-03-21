package frc.robot.subsystems.coral;

import static frc.robot.GlobalConstants.*;

public final class CoralPivotConstants {
  public static final int PIVOT_ID = 51;
  public static final boolean INVERTED = true;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = false;
  public static final double FORWARD_LIMIT = 0.5, REVERSE_LIMIT = 0.05;

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final double kP = 4.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final Gains GAINS = new Gains(kP, kI, kD, kS, kV, kA, 0.0);
}
