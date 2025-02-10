package frc.robot.generic.rollers;

import static frc.robot.Config.Subsystems.ALGAE_INTAKE_ENABLED;
import static frc.robot.Config.Subsystems.CORAL_INTAKE_ENABLED;
import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.algaeintake.AlgaeIntakeIOMax;
import frc.robot.subsystems.algaeintake.AlgaeIntakeIOSim;
import frc.robot.subsystems.algaeintake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.coralintake.CoralIntakeIOMax;
import frc.robot.subsystems.coralintake.CoralIntakeIOSim;
import frc.robot.subsystems.coralintake.CoralIntakeSubsystem;
import lombok.Getter;

public class Rollers extends SubsystemBase {
  @Getter
  private final AlgaeIntakeSubsystem algaeIntake =
      ALGAE_INTAKE_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new AlgaeIntakeSubsystem("Feeder", new AlgaeIntakeIOMax())
              : new AlgaeIntakeSubsystem(
                  "Feeder Sim", new AlgaeIntakeIOSim(new DCMotor(1, 1, 1, 1, 1, 1), 0, 0)))
          : null;

  @Getter
  private final CoralIntakeSubsystem coralIntake =
      ALGAE_INTAKE_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new CoralIntakeSubsystem("Feeder", new CoralIntakeIOMax())
              : new CoralIntakeSubsystem(
                  "Feeder Sim", new CoralIntakeIOSim(new DCMotor(1, 1, 1, 1, 1, 1), 0, 0)))
          : null;

  @Override
  public void periodic() {
    if (ALGAE_INTAKE_ENABLED) algaeIntake.periodic();
    if (CORAL_INTAKE_ENABLED) coralIntake.periodic();
  }
}
