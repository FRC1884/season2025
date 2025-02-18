package frc.robot.generic.elevators;

import static frc.robot.Config.Subsystems.CLIMBER_ENABLED;
import static frc.robot.Config.Subsystems.ELEVATOR_ENABLED;
import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIOFlex;
import frc.robot.subsystems.climber.ClimberIOMax;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOFlex;
import frc.robot.subsystems.elevator.ElevatorIOMax;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import lombok.Getter;

public class Elevators extends SubsystemBase {
  @Getter
  private final ClimberSubsystem climber =
      Config.Subsystems.CLIMBER_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? ClimberConstants.isFlex
                  ? new ClimberSubsystem("Climber", new ClimberIOFlex())
                  : new ClimberSubsystem(
                      "Climber",
                      new ClimberIOMax(
                          new int[] {ClimberConstants.RIGHT_CLIMBER, ClimberConstants.LEFT_CLIMBER},
                          new boolean[] {
                            ClimberConstants.RIGHT_INVERTED, ClimberConstants.LEFT_INVERTED
                          },
                          80,
                          true))
              : new ClimberSubsystem("Climber Sim", new ClimberIOSim()))
          : null;

  @Getter
  private final ElevatorSubsystem elevator =
      Config.Subsystems.ELEVATOR_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? ElevatorConstants.isFlex
                  ? new ElevatorSubsystem("Elevator", new ElevatorIOFlex())
                  : new ElevatorSubsystem("Elevator", new ElevatorIOMax())
              : new ElevatorSubsystem("Elevator Sim", new ElevatorIOSim(2, 0.0)))
          : null;

  @Override
  public void periodic() {
    if (ELEVATOR_ENABLED) elevator.periodic();
    if (CLIMBER_ENABLED) climber.periodic();
  }
}
