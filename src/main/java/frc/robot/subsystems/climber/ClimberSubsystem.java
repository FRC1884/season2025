package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ClimberSubsystem extends SubsystemBase {

  public final ClimberIO io;
  public final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

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

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
