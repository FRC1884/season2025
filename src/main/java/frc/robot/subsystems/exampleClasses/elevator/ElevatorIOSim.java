// change between sim and sparkmax files

package frc.robot.subsystems.exampleClasses.elevator;

import frc.robot.generic.elevators.GenericElevatorSystemIOSim;

public class ElevatorIOSim extends GenericElevatorSystemIOSim implements ElevatorIO {
  public ElevatorIOSim(int numMotors, double startingAngle) {
    super(numMotors, startingAngle);
  }
}
