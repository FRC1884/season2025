package frc.robot.generic.arm;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generic.rollers.Rollers;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public abstract class GenericPositionArmSystem<G extends GenericPositionArmSystem.PivotGoal> {
  public interface PivotGoal {
    DoubleSupplier getAngleSupplier();
  }

  public abstract G getGoal();

  private final String name;
  public final GenericArmSystemIO io;
  protected final GenericArmSystemIOInputsAutoLogged inputs =
      new GenericArmSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  public GenericPositionArmSystem(String name, GenericArmSystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  /**
   * NOT the same as {@link edu.wpi.first.wpilibj2.command.Subsystem#periodic()}. This method will
   * be called periodically in {@link Rollers}, hence why this subsystem does not extend {@link
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

    io.runToDegree(getGoal().getAngleSupplier().getAsDouble());

    Logger.recordOutput("Arm/" + name + "Goal", getGoal().toString());
  }
}
