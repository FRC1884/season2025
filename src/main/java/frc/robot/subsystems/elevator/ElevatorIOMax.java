package frc.robot.subsystems.elevator;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkMax;

public class ElevatorIOMax extends GenericElevatorSystemIOSparkMax implements ElevatorIO {
  public ElevatorIOMax() {
    super(
        new int[] {ElevatorConstants.LEFT_ELEVATOR, ElevatorConstants.RIGHT_ELEVATOR},
        40,
        0.0,
        false,
        true,
        1.0,
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD);
  }
}
