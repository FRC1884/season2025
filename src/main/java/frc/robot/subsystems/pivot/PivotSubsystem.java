package frc.robot.subsystems.pivot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase {

    @RequiredArgsConstructor
    public enum PivotGoal {
        IDLING(() -> 0.0),
        LEVEL_ONE(() -> 0.1),
        LEVEL_TWO(() -> 0.2),
        LEVEL_THREE(() -> 0.3),
        LEVEL_FOUR(() -> 0.4);

        private final DoubleSupplier heightSupplier;

        public double getHeightSupplier() {
            return heightSupplier.getAsDouble();
        }
    }

    private PivotGoal goal = PivotGoal.IDLING;
    private Debouncer currentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);
}
