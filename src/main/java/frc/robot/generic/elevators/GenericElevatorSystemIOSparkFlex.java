package frc.robot.generic.elevators;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class GenericElevatorSystemIOSparkFlex implements GenericElevatorSystemIO {
  private final SparkFlex[] motors;
  private final RelativeEncoder encoder;
  private SparkBaseConfig config;
  private SparkClosedLoopController controller;
  private double goal;
  private DoubleSupplier kpUp, kiUp, kdUp, kpDown, kiDown, kdDown;
  private boolean[] inverted;
  private boolean firstRun;
  private boolean limitSwitch = false;

  public GenericElevatorSystemIOSparkFlex(
      int[] id,
      boolean[] inverted,
      int currentLimitAmps,
      double restingAngle,
      boolean brake,
      double reduction,
      DoubleSupplier kPUp,
      DoubleSupplier kIUp,
      DoubleSupplier kDUp,
      DoubleSupplier kPDown,
      DoubleSupplier kIDown,
      DoubleSupplier kDDown) {
    this.kpUp = kPUp;
    this.kiUp = kIUp;
    this.kdUp = kDUp;
    this.kpDown = kPDown;
    this.kiDown = kIDown;
    this.kdDown = kDDown;
    this.inverted = inverted;
    motors = new SparkFlex[id.length];
    config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    config.closedLoop.pid(kPUp.getAsDouble(), kIUp.getAsDouble(), kDUp.getAsDouble());

    for (int i = 0; i < id.length; i++) {
      motors[i] = new SparkFlex(id[i], SparkLowLevel.MotorType.kBrushless);

      if (i == 0)
        motors[i].configure(
            config.inverted(inverted[i]),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
      else
        motors[i].configure(
            (new SparkFlexConfig().apply(config).follow(motors[0], inverted[i])),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
    }
    encoder = motors[0].getEncoder();
    controller = motors[0].getClosedLoopController();
  }

  @Override
  public void updateInputs(GenericElevatorSystemIOInputs inputs) {
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    inputs.appliedVoltage = motors[0].getAppliedOutput() * motors[0].getBusVoltage();
    inputs.supplyCurrentAmps = motors[0].getOutputCurrent();
    inputs.tempCelsius = motors[0].getMotorTemperature();
    inputs.goal = goal;
    inputs.limitSwitch = limitSwitch;

    Logger.recordOutput("Elevator/AtGoal", Math.abs(goal - encoder.getPosition()) < 0.5);
  }

  @Override
  public void runPosition(double position) {
    if (firstRun) {
      if (position > encoder.getPosition()) {
        config.closedLoop.pid(kpUp.getAsDouble(), kiUp.getAsDouble(), kdUp.getAsDouble());
        motors[0].configure(
            config.inverted(inverted[0]),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
        firstRun = false;
      } else if (position < encoder.getPosition()) {
        config.closedLoop.pid(kpDown.getAsDouble(), kiDown.getAsDouble(), kdDown.getAsDouble());
        motors[0].configure(
            config.inverted(inverted[0]),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
        firstRun = false;
      }
    }
    controller.setReference(position, ControlType.kPosition);
    goal = position;
  }

  @Override
  public void resetEncoder(boolean limitSwitch) {
    // encoder.setPosition(0.0);
    this.limitSwitch = limitSwitch;
  }

  @Override
  public void updatePID() {
    firstRun = true;
  }
}
