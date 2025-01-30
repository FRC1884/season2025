// change between sim and sparkmax files

package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSim;

public class ClimberIOSim extends GenericElevatorSystemIOSim implements ClimberIO {
  public ClimberIOSim(int numMotors, double startingAngle) {
    super(numMotors, startingAngle);
  }
}
