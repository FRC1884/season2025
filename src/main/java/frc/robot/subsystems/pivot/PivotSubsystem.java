package frc.robot.subsystems.pivot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.generic.arm.GenericPositionArmSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class PivotSubsystem extends GenericPositionArmSystem<PivotSubsystem.PivotGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum PivotGoal implements GenericPositionArmSystem.PivotGoal {
    IDLING(new LoggedTunableNumber("Pivot/RestingAngle", 0)),
    LEVEL_ONE(() -> PivotConstants.Hardware.RESTING_ANGLE + 10),
    LEVEL_TWO(() -> PivotConstants.Hardware.RESTING_ANGLE + 20),
    LEVEL_THREE(() -> PivotConstants.Hardware.RESTING_ANGLE + 30),
    LEVEL_FOUR(new LoggedTunableNumber("Pivot/L4Angle", 50));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngleSupplier() {
      return angleSupplier;
    }
  }

  private PivotGoal goal = PivotGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public PivotSubsystem(String name, PivotIO io) {
    super(name, io);
  }
}
