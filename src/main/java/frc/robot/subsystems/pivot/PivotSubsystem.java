package frc.robot.subsystems.pivot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class PivotSubsystem extends SubsystemBase {

  public final PivotIO io;
  protected final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  @RequiredArgsConstructor
  public enum PivotGoal {
    IDLING(() -> 0.0),
    LEVEL_ONE(() -> 0.1),
    LEVEL_TWO(() -> 0.2),
    LEVEL_THREE(() -> 0.3),
    LEVEL_FOUR(() -> 0.4),
    SOURCE(() -> 0.5);

    private final DoubleSupplier heightSupplier;

    public double getHeightSupplier() {
      return heightSupplier.getAsDouble();
    }
  }

  private PivotGoal goal = PivotGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  public PivotSubsystem(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
