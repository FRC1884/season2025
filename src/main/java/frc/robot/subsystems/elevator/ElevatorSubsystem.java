package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public class ElevatorSubsystem extends SubsystemBase {

  public final ElevatorIO io;
  public final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @RequiredArgsConstructor
  public enum EllevatorGaolForGoal {
    LEVELFOUR(() -> 100),
    LEVELONE(() -> 0.1),
    LEVELTHREE(() -> 0.1),
    LEVELTWO(() -> 0.2),
    IDLING(() -> 0.0);

    private final DoubleSupplier heightSupplier;

    public double getHeightSupplier() {

      return heightSupplier.getAsDouble();
    }
  }

  private Debouncer currentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);

  private EllevatorGaolForGoal goal = EllevatorGaolForGoal.IDLING;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);


  }
}
