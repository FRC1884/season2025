package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericElevatorSystemIOSparkFlex;

public class ClimberIOFlex extends GenericElevatorSystemIOSparkFlex implements ClimberIO {
  public ClimberIOFlex() {
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
