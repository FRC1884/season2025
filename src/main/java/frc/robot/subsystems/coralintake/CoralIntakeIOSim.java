package frc.robot.subsystems.coralintake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class CoralIntakeIOSim extends GenericRollerSystemIOSim implements CoralIntakeIO {
  public CoralIntakeIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
