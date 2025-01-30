package frc.robot.subsystems.pivot;

import frc.robot.generic.arm.GenericArmSystemIOSparkMax;

public class PivotIOFlex extends GenericArmSystemIOSparkMax implements PivotIO {
  public PivotIOFlex() {
    super(
        new int[] {PivotConstants.Software.LEFT_PIVOT_ID, PivotConstants.Software.RIGHT_PIVOT_ID},
        40,
        PivotConstants.Hardware.RESTING_ANGLE,
        false,
        true,
        1.0,
        PivotConstants.Software.gains.kP(),
        PivotConstants.Software.gains.kI(),
        PivotConstants.Software.gains.kD());
  }
}
