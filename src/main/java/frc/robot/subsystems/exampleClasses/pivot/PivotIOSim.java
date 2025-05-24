// change between sim and sparkmax files

package frc.robot.subsystems.exampleClasses.pivot;

import frc.robot.generic.arms.GenericArmSystemIOSim;

public class PivotIOSim extends GenericArmSystemIOSim implements PivotIO {
  public PivotIOSim(int numMotors, double startingAngle) {
    super(numMotors, startingAngle);
  }
}
