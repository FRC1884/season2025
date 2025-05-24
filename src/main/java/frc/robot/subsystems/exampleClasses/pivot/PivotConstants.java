package frc.robot.subsystems.exampleClasses.pivot;

import frc.robot.GlobalConstants;

public final class PivotConstants {
  public static final int PIVOT_ID = 61; // TODO: find motor id
  public static final boolean INVERTED = true;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = false;
  public static final double FORWARD_LIMIT = 1.0, REVERSE_LIMIT = 0.0; // TODO: find limits

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final double kP = 4.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains(kP, kI, kD, kS, kV, kA, 0.0);
}
