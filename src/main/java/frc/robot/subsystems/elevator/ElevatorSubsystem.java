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
    IDLING(new LoggedTunableNumber("Elevator/Idling", 0.0)),
    LEVEL_ONE(new LoggedTunableNumber("Elevator/Level_One", 0.46)),
    LEVEL_TWO(new LoggedTunableNumber("Elevator/Level_Two", 0.81)),
    LEVEL_THREE(new LoggedTunableNumber("Elevator/Level_Three", 1.21)),
    LEVEL_FOUR(new LoggedTunableNumber("Elevator/Level_Four", 1.83)),
    TESTING(new LoggedTunableNumber("Elevator/Test", 0.0));

    private final DoubleSupplier heightSupplier;

    @Override
    public DoubleSupplier getHeight() {
      return () -> (heightSupplier.getAsDouble() + ElevatorConstants.restingRot);
    }
  }

  private ElevatorGoal goal = ElevatorGoal.IDLING;

  public ElevatorSubsystem(String name, ElevatorIO io) {
    super(name, io);
  }
}
