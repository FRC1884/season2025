package frc.robot.subsystems.algaeintake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generic.rollers.GenericRollerSystemIOSim;

public class AlgaeIntakeIOSim extends GenericRollerSystemIOSim implements AlgaeIntakeIO {
  public AlgaeIntakeIOSim(DCMotor motorModel, double reduction, double moi) {
    super(motorModel, reduction, moi);
  }
}
