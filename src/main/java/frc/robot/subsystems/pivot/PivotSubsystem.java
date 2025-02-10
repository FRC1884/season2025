package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.RESTING_ANGLE;

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
    IDLING(new LoggedTunableNumber("Pivot/RestingAngle", RESTING_ANGLE)),
    LEVEL_ONE(new LoggedTunableNumber("Pivot/Level1Angle", RESTING_ANGLE + 10)),
    LEVEL_TWO(new LoggedTunableNumber("Pivot/Leve2Angle", RESTING_ANGLE + 30)),
    LEVEL_THREE(new LoggedTunableNumber("Pivot/Level3Angle", RESTING_ANGLE + 30)),
    LEVEL_FOUR(new LoggedTunableNumber("Pivot/Level4Angle", RESTING_ANGLE + 50)),
    TESTING(new LoggedTunableNumber("Pivot/Level4Angle", 0.0));


    private final DoubleSupplier angleSupplier;
  }

  private PivotGoal goal = PivotGoal.IDLING;

  public PivotSubsystem(String name, PivotIO io) {
    super(name, io);
  }
}
