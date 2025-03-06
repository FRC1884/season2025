package frc.robot.generic.arm;

import static frc.robot.Config.Subsystems.ALGAE_PIVOT_ENABLED;
import static frc.robot.Config.Subsystems.CORAL_PIVOT_ENABLED;
import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.algae.AlgaePivotConstants;
import frc.robot.subsystems.algae.AlgaePivotIOFlex;
import frc.robot.subsystems.algae.AlgaePivotIOMax;
import frc.robot.subsystems.algae.AlgaePivotIOSim;
import frc.robot.subsystems.algae.AlgaePivotSubsystem;
import frc.robot.subsystems.coral.CoralPivotConstants;
import frc.robot.subsystems.coral.CoralPivotIOFlex;
import frc.robot.subsystems.coral.CoralPivotIOMax;
import frc.robot.subsystems.coral.CoralPivotIOSim;
import frc.robot.subsystems.coral.CoralPivotSubsystem;
import lombok.Getter;

public class Arms extends SubsystemBase {
  @Getter
  private final CoralPivotSubsystem coralPivot =
      CORAL_PIVOT_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? CoralPivotConstants.IS_FLEX
                  ? new CoralPivotSubsystem("CoralPivot", new CoralPivotIOFlex())
                  : new CoralPivotSubsystem("CoralPivot", new CoralPivotIOMax())
              : new CoralPivotSubsystem("Coral Pivot Sim", new CoralPivotIOSim(2, 0.0)))
          : null;

  @Getter
  private final AlgaePivotSubsystem algaePivot =
      Config.Subsystems.ALGAE_PIVOT_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? AlgaePivotConstants.IS_FLEX
                  ? new AlgaePivotSubsystem("AlgaePivot", new AlgaePivotIOFlex())
                  : new AlgaePivotSubsystem("AlgaePivot", new AlgaePivotIOMax())
              : new AlgaePivotSubsystem("Algae Pivot Sim", new AlgaePivotIOSim(2, 0.0)))
          : null;

  @Override
  public void periodic() {
    if (CORAL_PIVOT_ENABLED) coralPivot.periodic();
    if (ALGAE_PIVOT_ENABLED) algaePivot.periodic();
  }
}
