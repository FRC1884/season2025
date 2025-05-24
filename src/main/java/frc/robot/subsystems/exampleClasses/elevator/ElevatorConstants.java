package frc.robot.subsystems.exampleClasses.elevator;

import frc.robot.GlobalConstants;

public final class ElevatorConstants {
  public static final int LEFT_ELEVATOR = 31; // TODO: Change to correct Motor ID's
  public static final boolean LEFT_INVERTED = true;
  public static final int RIGHT_ELEVATOR = 32; // TODO: Change to correct Motor ID's
  public static final boolean RIGHT_INVERTED = true;
  public static final int DIOPort = 0;

  public static final boolean isFlex = true; // if the motors on the Elevator are flex motors

  // Tuned in REV Hardware Client for real bots, but should we use LTNs?
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0;
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains(kP, kI, kD, kS, kV, kA, kG);
}
