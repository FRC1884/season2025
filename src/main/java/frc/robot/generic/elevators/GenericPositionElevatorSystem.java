package frc.robot.generic.elevators;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalConstants;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public abstract class GenericPositionElevatorSystem<
        G extends GenericPositionElevatorSystem.ExtensionGoal>
    extends SubsystemBase {
  // Goals
  public interface ExtensionGoal {
    DoubleSupplier getHeight();
  }

  public abstract G getGoal();

  private final String name;
  public final GenericElevatorSystemIO io;
  public final GenericElevatorSystemIOInputsAutoLogged inputs =
      new GenericElevatorSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  // Movement
  private final PIDController pidController;
  private final ElevatorFeedforward feedforward;
  private final SysIdRoutine sysIdRoutine;
  @Getter private double goalPosition;

  private double kS, kG, kV, kA;

  public GenericPositionElevatorSystem(
      String name, GenericElevatorSystemIO io, GlobalConstants.Gains gains) {
    this.name = name;
    this.io = io;
    this.pidController = new PIDController(gains.kP(), gains.kI(), gains.kD());
    this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    this.kS = gains.kS();
    this.kG = gains.kG();
    this.kV = gains.kV();
    this.kA = gains.kA();

    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state ->
                    Logger.recordOutput("Elevators/" + name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(voltage -> io.setVoltage(voltage.in(Volts)), null, this));

    disconnected = new Alert("Motor(s) disconnected on arm: " + name + "!", Alert.AlertType.kError);
    stateTimer.start();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  /**
   * This method is called periodically, and is responsible for updating the state of the arm
   * subsystem. It reads the current state of the arm and logs it to the advantage scope, checks for
   * any disconnected motors, and applies the PID control to the arm.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    boolean anyDisconnected = false;
    for (boolean isConnected : inputs.connected) {
      if (!isConnected) {
        anyDisconnected = true;
        break;
      }
    }
    disconnected.set(anyDisconnected);

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    double currentPosition = inputs.encoderPosition;
    double velocity = inputs.velocity;

    double pidOutput = pidController.calculate(currentPosition, goalPosition);
    double feedforwardOutput = feedforward.calculate(velocity);
    double outputVoltage = pidOutput + feedforwardOutput;

    io.setVoltage(outputVoltage);

    Logger.recordOutput("Elevators/" + name + "Goal", getGoal().toString());
    Logger.recordOutput("Elevators/" + name + "/Feedforward", feedforwardOutput);
  }
}
