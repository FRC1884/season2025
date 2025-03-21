package frc.robot.subsystems.coral;

import static frc.robot.subsystems.coral.CoralPivotConstants.FORWARD_LIMIT;
import static frc.robot.subsystems.coral.CoralPivotConstants.REVERSE_LIMIT;

import frc.robot.generic.arms.GenericArmSystemIOSparkFlex;

public class CoralPivotIOFlex extends GenericArmSystemIOSparkFlex implements CoralPivotIO {

  public CoralPivotIOFlex() {
    super(new int[] {CoralPivotConstants.PIVOT_ID}, 40, true, FORWARD_LIMIT, REVERSE_LIMIT);

    invert(0);
  }
}
