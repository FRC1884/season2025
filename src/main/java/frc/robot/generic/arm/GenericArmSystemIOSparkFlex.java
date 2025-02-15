package frc.robot.generic.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class GenericArmSystemIOSparkFlex implements GenericArmSystemIO {
  private final SparkFlex[] motors;
  private final RelativeEncoder encoder;
  private double restingAngle;
  private final double reduction;
  private SparkBaseConfig config;
  private SparkClosedLoopController controller;
  private double goal;
  private DoubleSupplier kp, ki, kd;
  private boolean[] inverted;

  public GenericArmSystemIOSparkFlex(
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
    this.reduction = reduction;
    this.restingAngle = restingAngle;
    motors = new SparkFlex[id.length];
    config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    config
        .closedLoop
        .pid(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble())
        .maxMotion
        .maxAcceleration(6000)
        .maxVelocity(6000)
        .allowedClosedLoopError(0.2);

    for (int i = 0; i < id.length; i++) {
      motors[i] = new SparkFlex(id[i], SparkLowLevel.MotorType.kBrushless);

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

  public void updateInputs(GenericArmSystemIOInputs inputs) {
    if (motors != null) {
      inputs.degrees = Units.rotationsToDegrees(encoder.getPosition());
      inputs.rotations = motors[0].getEncoder().getPosition();
      inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
      inputs.appliedVoltage = motors[0].getAppliedOutput() * motors[0].getBusVoltage();
      inputs.supplyCurrentAmps = motors[0].getOutputCurrent();
      inputs.tempCelsius = motors[0].getMotorTemperature();
      inputs.goal = goal;
    }
  }

  @Override
  public void runToDegree(double position) {
    config.closedLoop.pid(kp.getAsDouble(), ki.getAsDouble(), kd.getAsDouble());
    motors[0].configure(
        config.inverted(inverted[0]),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    controller.setReference(position, ControlType.kMAXMotionPositionControl);
    goal = position;
  }
}
