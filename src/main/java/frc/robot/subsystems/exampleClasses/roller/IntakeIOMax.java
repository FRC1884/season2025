package frc.robot.subsystems.exampleClasses.roller;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class IntakeIOMax extends GenericRollerSystemIOSparkMax implements IntakeIO {
  public IntakeIOMax() {
    super(IntakeConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
