package frc.robot.subsystems.elevator;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkFlex;

public class ElevatorIOFlex extends GenericElevatorSystemIOSparkFlex implements ElevatorIO {
  public ElevatorIOFlex() {
    super(
        new int[] {ElevatorConstants.LEFT_ELEVATOR, ElevatorConstants.RIGHT_ELEVATOR},
        new boolean[] {ElevatorConstants.LEFT_INVERTED, ElevatorConstants.RIGHT_INVERTED},
        40,
        0.0,
        true,
        1.0,
        ElevatorConstants.kPUp,
        ElevatorConstants.kIUp,
        ElevatorConstants.kDUp,
        ElevatorConstants.kPDown,
        ElevatorConstants.kIDown,
        ElevatorConstants.kDDown);
  }
}
