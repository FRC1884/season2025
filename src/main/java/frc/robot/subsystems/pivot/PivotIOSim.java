// change between sim and sparkmax files

package frc.robot.subsystems.pivot;

import frc.robot.generic.arm.GenericArmSystemIOSim;

public class PivotIOSim extends GenericArmSystemIOSim implements PivotIO {
  public PivotIOSim(int numMotors, double startingAngle) {
    super(numMotors, startingAngle);
  }
}
