// change between sim and sparkmax files

package frc.robot.subsystems.algae;

import frc.robot.generic.arms.GenericArmSystemIOSim;

public class AlgaePivotIOSim extends GenericArmSystemIOSim implements AlgaePivotIO {
  public AlgaePivotIOSim(int numMotors, double startingAngle) {
    super(numMotors, startingAngle);
  }
}
