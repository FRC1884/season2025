package frc.robot.subsystems.elevator;

import frc.robot.generic.elevators.GenericPositionElevatorSystem;
import frc.robot.generic.elevators.GenericPositionElevatorSystem.ExtensionGoal;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ElevatorSubsystem
    extends GenericPositionElevatorSystem<ElevatorSubsystem.ElevatorGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ElevatorGoal implements ExtensionGoal {
    IDLING(() -> 3),
    SOURCE(() -> 5),
    LEVEL_ONE(() -> 8),
    LEVEL_TWO(() -> 14),
    LEVEL_THREE(() -> 24),
    LEVEL_FOUR(() -> 41),
    TESTING(new LoggedTunableNumber("Elevator/Test", 10.0));

    private final DoubleSupplier heightSupplier;

    @Override
    public DoubleSupplier getHeight() {
      return () -> heightSupplier.getAsDouble();
    }
  }

  private ElevatorGoal goal = ElevatorGoal.IDLING;

  public ElevatorSubsystem(String name, ElevatorIO io) {
    super(name, io);
  }
}
