package frc.robot.subsystems.exampleClasses.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem {
  @RequiredArgsConstructor
  @Getter
  public enum ClimberGoal {
    IDLING(() -> 0.0), // Intake is off
    TESTING(new LoggedTunableNumber("Climber/Test", 0.0));

    private final DoubleSupplier voltageSupplier;

  }

  @Getter @Setter private ClimberGoal goal = ClimberGoal.IDLING;
  @Getter private ClimberGoal lastGoal = ClimberGoal.IDLING;
  private final String name;
  public final ClimberIO io;
  protected final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();

  public ClimberSubsystem(String name, ClimberIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  /**
   * NOT the same as {@link edu.wpi.first.wpilibj2.command.Subsystem#periodic()}. This method will
   * be called periodically in {@link ClimberSubsystem}, hence why this subsystem does not extend {@link
   * SubsystemBase}.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    io.runVolts(getGoal().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Climber/" + name + "Goal", getGoal().toString());
  }
}
