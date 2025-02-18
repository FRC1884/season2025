package frc.robot.subsystems.algae;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.generic.rollers.GenericVoltageRollerSystem;
import frc.robot.generic.rollers.GenericVoltageRollerSystem.VoltageGoal;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class AlgaeIntakeSubsystem
    extends GenericVoltageRollerSystem<AlgaeIntakeSubsystem.AlgaeIntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum AlgaeIntakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 4), // Maximum forward voltage
    REVERSE(() -> -4); // Maximum reverse voltage

    private final DoubleSupplier voltageSupplier;
  }

  @Setter private AlgaeIntakeGoal goal = AlgaeIntakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public AlgaeIntakeSubsystem(String name, AlgaeIntakeIO io) {
    super(name, io);
  }

  public BooleanSupplier hasAlgae() {
    return () ->
        (goal == AlgaeIntakeGoal.FORWARD
            && stateTimer.hasElapsed(0.25)
            && currentDebouncer.calculate(inputs.torqueCurrentAmps > 45.0));
  }
}
