package frc.robot.subsystems.coral;

import static frc.robot.subsystems.coral.CoralPivotConstants.FORWARD_LIMIT;
import static frc.robot.subsystems.coral.CoralPivotConstants.REVERSE_LIMIT;

import frc.robot.generic.arm.GenericArmSystemIOSparkMax;

public class CoralPivotIOMax extends GenericArmSystemIOSparkMax implements CoralPivotIO {

  public CoralPivotIOMax() {
    super(
        new int[] {CoralPivotConstants.PIVOT_ID},
        new boolean[] {CoralPivotConstants.INVERTED},
        40,
        false,
        FORWARD_LIMIT,
        REVERSE_LIMIT,
        CoralPivotConstants.kP);
  }
}
