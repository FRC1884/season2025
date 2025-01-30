package frc.robot.generic.elevators;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public abstract class GenericPositionElevatorSystem<
    G extends GenericPositionElevatorSystem.ExtensionGoal> {
  public interface ExtensionGoal {
    DoubleSupplier getHeight();
  }

  public abstract G getGoal();

  private final String name;
  public final GenericElevatorSystemIO io;
  protected final GenericElevatorSystemIOInputsAutoLogged inputs =
      new GenericElevatorSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  public GenericPositionElevatorSystem(String name, GenericElevatorSystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  /**
   * NOT the same as {@link edu.wpi.first.wpilibj2.command.Subsystem#periodic()}. This method will
   * be called periodically in {@link Elevators}, hence why this subsystem does not extend {@link
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

    io.runPosition(getGoal().getHeight().getAsDouble());
    Logger.recordOutput("Elevators/" + name + "Goal", getGoal().toString());
  }
}
