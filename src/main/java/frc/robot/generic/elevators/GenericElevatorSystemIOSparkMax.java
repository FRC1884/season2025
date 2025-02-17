package frc.robot.generic.elevators;

import static frc.robot.GlobalConstants.TUNING_MODE;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.function.DoubleSupplier;

public class GenericElevatorSystemIOSparkMax implements GenericElevatorSystemIO {
  private final SparkMax[] motors;
  private final RelativeEncoder encoder;
  private SparkBaseConfig config;
  private SparkClosedLoopController controller;
  private double goal;
  private DoubleSupplier kp, ki, kd;
  private boolean[] inverted;

  public GenericElevatorSystemIOSparkMax(
      int[] id,
      boolean[] inverted,
      int currentLimitAmps,
      double restingAngle,
      boolean brake,
      double reduction,
      DoubleSupplier kP,
      DoubleSupplier kI,
      DoubleSupplier kD) {
    this.kp = kP;
    this.ki = kI;
    this.kd = kD;
    this.inverted = inverted;
    motors = new SparkMax[id.length];
    config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    config.closedLoop.pid(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());

    for (int i = 0; i < id.length; i++) {
      motors[i] = new SparkMax(id[i], SparkLowLevel.MotorType.kBrushless);

      if (i == 0)
        motors[i].configure(
            config.inverted(inverted[i]),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      else
        motors[i].configure(
            new SparkFlexConfig().apply(config).follow(motors[0], inverted[i]),
            ResetMode.kResetSafeParameters,
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
  }

  @Override
  public void runPosition(double position) {
    if (TUNING_MODE) {
      config.closedLoop.pid(kp.getAsDouble(), ki.getAsDouble(), kd.getAsDouble());
      motors[0].configure(
          config.inverted(inverted[0]),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
    controller.setReference(position, ControlType.kPosition);
    goal = position;
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0.0);
  }
}
