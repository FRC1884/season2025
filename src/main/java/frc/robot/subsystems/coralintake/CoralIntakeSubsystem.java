package frc.robot.subsystems.coralintake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.generic.rollers.GenericVoltageRollerSystem;
import frc.robot.util.LoggedTunableNumber;
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
    IDLING(new LoggedTunableNumber("Feeder/IdleVoltage", 0.0)), // Intake is off
    FORWARD(new LoggedTunableNumber("Feeder/ForwardVoltage", 12.0)), // Maximum forward voltage
    REVERSE(new LoggedTunableNumber("Feeder/ReverseVoltage", -12.0)); // Maximum reverse voltage

    private final DoubleSupplier voltageSupplier;
  }

  @Setter private CoralIntakeGoal goal = CoralIntakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  public CoralIntakeSubsystem(String name, CoralIntakeIO io) {
    super(name, io);
  }

  public BooleanSupplier hasCoral() {
    return () ->
        (goal == CoralIntakeGoal.FORWARD
            && stateTimer.hasElapsed(0.25)
            && currentDebouncer.calculate(inputs.torqueCurrentAmps > 45.0));
  }
}
