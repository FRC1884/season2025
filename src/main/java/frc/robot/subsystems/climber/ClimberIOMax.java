package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkMax;

public class ClimberIOMax extends GenericElevatorSystemIOSparkMax implements ClimberIO {
  public ClimberIOMax() {
    super(
        new int[] {ClimberConstants.LEFT_CLIMBER, ClimberConstants.RIGHT_CLIMBER},
        40,
        0.0,
        false,
        true,
        1.0,
        ClimberConstants.kP,
        ClimberConstants.kI,
        ClimberConstants.kD);
  }
}
