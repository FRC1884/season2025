package frc.robot.subsystems.algae;

import static frc.robot.subsystems.algae.AlgaePivotConstants.FORWARD_LIMIT;
import static frc.robot.subsystems.algae.AlgaePivotConstants.REVERSE_LIMIT;

import frc.robot.generic.arms.GenericArmSystemIOSparkMax;

public class AlgaePivotIOMax extends GenericArmSystemIOSparkMax implements AlgaePivotIO {

  public AlgaePivotIOMax() {
    super(new int[] {AlgaePivotConstants.PIVOT_ID}, 40, true, FORWARD_LIMIT, REVERSE_LIMIT);

    invert(0);
  }
}
