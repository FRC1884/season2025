package frc.robot.subsystems.algaeintake;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class AlgaeIntakeIOMax extends GenericRollerSystemIOSparkMax implements AlgaeIntakeIO {
  public AlgaeIntakeIOMax() {
    super(AlgaeIntakeConstants.INTAKE_ALGAE_ID, 40, false, true, 0.0);
  }
}
