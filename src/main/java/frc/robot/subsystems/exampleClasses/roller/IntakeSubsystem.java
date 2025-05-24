package frc.robot.subsystems.exampleClasses.roller;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.generic.rollers.GenericVoltageRollerSystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class IntakeSubsystem
    extends GenericVoltageRollerSystem<IntakeSubsystem.AlgaeIntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum AlgaeIntakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 6), // Maximum forward voltage
    REVERSE(() -> -8); // Maximum reverse voltage

    private final DoubleSupplier voltageSupplier;
  }

  @Setter private AlgaeIntakeGoal goal = AlgaeIntakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);

  public IntakeSubsystem(String name, IntakeIO io) {
    super(name, io);
  }

  public BooleanSupplier hasAlgae() {
    return () ->
        (goal == AlgaeIntakeGoal.FORWARD
            && currentDebouncer.calculate(
                inputs.torqueCurrentAmps > IntakeConstants.currentLimit.getAsDouble()));
  }
}
