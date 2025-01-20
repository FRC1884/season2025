package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

public class ClimberSubsystem extends SubsystemBase {

    @RequiredArgsConstructor
    public enum ClimberGoal {
        IDLING(() -> 0.0),
        DEEP_CLIMB(() -> 0.1),
        SHALLOW_CLIMB(() -> 0.2);

        private final DoubleSupplier heightSupplier;

        public double getHeightSupplier() {
            return heightSupplier.getAsDouble();
        }
    }

    private ClimberGoal goal = ClimberGoal.IDLING;
    private Debouncer currentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);
}
