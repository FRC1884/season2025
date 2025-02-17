package frc.robot.subsystems.coral;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.generic.rollers.GenericVoltageRollerSystem;
import frc.robot.generic.rollers.GenericVoltageRollerSystem.VoltageGoal;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class CoralIntakeSubsystem
    extends GenericVoltageRollerSystem<CoralIntakeSubsystem.CoralIntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum CoralIntakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 12.0), // Maximum forward voltage
    REVERSE(() -> -12.0), // Maximum reverse voltage
    LFOUTAKE(() -> -4);
    private final DoubleSupplier voltageSupplier;
  }

  @Setter private CoralIntakeGoal goal = CoralIntakeGoal.IDLING;
  private Debouncer debouncer = new Debouncer(0.1);

  public CoralIntakeSubsystem(String name, CoralIntakeIO io) {
    super(name, io);
  }

  public BooleanSupplier hasCoral() {
    return () ->
        (goal == CoralIntakeGoal.FORWARD
            && debouncer.calculate(
                inputs.torqueCurrentAmps > CoralIntakeConstants.currentLimit.getAsDouble()));
  }
}
