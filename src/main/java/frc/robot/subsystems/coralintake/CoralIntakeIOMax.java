package frc.robot.subsystems.coralintake;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class CoralIntakeIOMax extends GenericRollerSystemIOSparkMax implements CoralIntakeIO {
  public CoralIntakeIOMax() {
    super(CoralIntakeConstants.INTAKE_CORAL_ID, 40, false, true, 0.0);
  }
}
