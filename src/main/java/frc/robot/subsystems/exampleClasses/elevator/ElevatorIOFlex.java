package frc.robot.subsystems.exampleClasses.elevator;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkFlex;

public class ElevatorIOFlex extends GenericElevatorSystemIOSparkFlex implements ElevatorIO {
  public ElevatorIOFlex() {
    super(new int[] {ElevatorConstants.LEFT_ELEVATOR, ElevatorConstants.RIGHT_ELEVATOR}, 40, true);

    invert(0);
  }
}
