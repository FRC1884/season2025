package frc.robot.generic.arms;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
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

public abstract class GenericPositionArmSystem<G extends GenericPositionArmSystem.PivotGoal>
    extends SubsystemBase {
  // Goals
  public interface PivotGoal {
    DoubleSupplier getAngle();
  }

  public abstract G getGoal();

  private final String name;
  public final GenericArmSystemIO io;
  protected final GenericArmSystemIOInputsAutoLogged inputs =
      new GenericArmSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  // Movement
  private final PIDController pidController;
  private final ArmFeedforward feedforward;
  private final SysIdRoutine sysIdRoutine;
  @Getter private double goalPosition;

  private double kS, kG, kV, kA;

  public GenericPositionArmSystem(String name, GenericArmSystemIO io, GlobalConstants.Gains gains) {
    this.name = name;
    this.io = io;
    this.pidController = new PIDController(gains.kP(), gains.kI(), gains.kD());
    this.feedforward = new ArmFeedforward(kS, kG, kV, kA);
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
                state -> Logger.recordOutput("Arms/" + name + "/SysIdState", state.toString())),
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
    double feedforwardOutput = feedforward.calculate(currentPosition, velocity);
    double outputVoltage = pidOutput + feedforwardOutput;

    io.setVoltage(outputVoltage);

    Logger.recordOutput("Arms/" + name + "/Goal", getGoal().toString());
    Logger.recordOutput("Arms/" + name + "/Feedforward", feedforwardOutput);
  }
}
