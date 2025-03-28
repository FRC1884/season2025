package frc.robot.subsystems.algae;

import frc.robot.generic.arm.GenericPositionArmSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class AlgaePivotSubsystem extends GenericPositionArmSystem<AlgaePivotSubsystem.PivotGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum PivotGoal implements GenericPositionArmSystem.PivotGoal {
    IDLING(() -> 0.07),
    L2(() -> 0.3),
    L3(() -> 0.3),
    BARGE(() -> 0.38),
    TESTING(new LoggedTunableNumber("AlgaePivot/TESTING", 0.1));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngle() {
      return () -> angleSupplier.getAsDouble();
    }
  }

  private PivotGoal goal = PivotGoal.IDLING;

  public AlgaePivotSubsystem(String name, AlgaePivotIO io) {
    super(name, io);
  }
}
