package frc.robot.subsystems.coral;

import frc.robot.generic.arm.GenericPositionArmSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class CoralPivotSubsystem extends GenericPositionArmSystem<CoralPivotSubsystem.PivotGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum PivotGoal implements GenericPositionArmSystem.PivotGoal {
    IDLING(() -> 0.48),
    SOURCE(() -> 0.33),
    LEVEL_ONE(() -> 0.24),
    LEVEL_TWO(() -> 0.195),
    LEVEL_THREE(() -> 0.205),
    LEVEL_FOUR(() -> 0.25),
    LEVEL_FOUR_FLICK(() -> 0.5),
    DEALGAEFY_L2(() -> 0.28),
    DEALGAEFY_L3(() -> 0.28),
    TESTING(new LoggedTunableNumber("Pivot/TESTING", 0.0));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngle() {
      return () -> angleSupplier.getAsDouble();
    }
  }

  private PivotGoal goal = PivotGoal.IDLING;

  public CoralPivotSubsystem(String name, CoralPivotIO io) {
    super(name, io);
  }
}
