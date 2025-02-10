package frc.robot.subsystems.pivot;

import static frc.robot.GlobalConstants.*;
import static frc.robot.GlobalConstants.ROBOT;

public final class PivotConstants {
  public static final int PIVOT_ID = 24;
  // Whether the motors on the Pivot are flex motors
  public static final boolean IS_FLEX = true;

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final Gains GAINS =
      switch (ROBOT) {
        case COMPBOT -> new Gains(0.02, 0, 0.0, 0, 0.5, 1.3); // 0.4 kP for NEO
        case DEVBOT -> new Gains(0, 0, 0, 0, 0, 0);
        case SIMBOT -> new Gains(0, 0, 0, 0, 0, 0);
      };
  public static final double MAX_VELOCITY = 6000;
  public static final double MAX_ACCELERATION = 6000;
  public static final double POSITION_TOLERANCE = 0.05;

  public static final double ABSOLUTE_ENCODER_OFFSET = 0.38;
  public static final double RESTING_ANGLE = 0.0;
}
