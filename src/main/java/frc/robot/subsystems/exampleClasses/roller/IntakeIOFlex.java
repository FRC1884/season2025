package frc.robot.subsystems.exampleClasses.roller;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkFlex;

public class IntakeIOFlex extends GenericRollerSystemIOSparkFlex implements IntakeIO {
  public IntakeIOFlex() {
    super(IntakeConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
