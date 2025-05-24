package frc.robot.subsystems.exampleClasses.pivot;

import static frc.robot.subsystems.exampleClasses.pivot.PivotConstants.FORWARD_LIMIT;
import static frc.robot.subsystems.exampleClasses.pivot.PivotConstants.REVERSE_LIMIT;

import frc.robot.generic.arms.GenericArmSystemIOSparkMax;

public class PivotIOMax extends GenericArmSystemIOSparkMax implements PivotIO {

  public PivotIOMax() {
    super(new int[] {PivotConstants.PIVOT_ID}, 40, true, FORWARD_LIMIT, REVERSE_LIMIT);

    invert(0);
  }
}
