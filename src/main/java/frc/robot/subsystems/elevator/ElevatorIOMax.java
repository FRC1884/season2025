package frc.robot.subsystems.elevator;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkMax;

public class ElevatorIOMax extends GenericElevatorSystemIOSparkMax implements ElevatorIO {
  public ElevatorIOMax() {
    super(
        new int[] {ElevatorConstants.RIGHT_ELEVATOR, ElevatorConstants.LEFT_ELEVATOR},
        new boolean[] {ElevatorConstants.RIGHT_INVERTED, ElevatorConstants.LEFT_INVERTED},
        40,
        0.0,
        true,
        1.0,
        ElevatorConstants.kPUp,
        ElevatorConstants.kIUp,
        ElevatorConstants.kDUp);
  }
}
