package frc.robot.subsystems.feeder;

import frc.robot.generic.rollers.GenericRollerSystemIOSparkMax;

public class FeederIOReal extends GenericRollerSystemIOSparkMax implements FeederIO {
  public FeederIOReal() {
    super(FeederConstants.FEEDER_ID, 40, false, true, 0.0);
  }
}
