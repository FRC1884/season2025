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

public class GenericArmSystemIOSparkFlex implements GenericArmSystemIO {
  private final SparkFlex[] motors;
  private final RelativeEncoder encoder;
  private double restingAngle;
  private final double reduction;
  private SparkBaseConfig config;
  private SparkClosedLoopController controller;

  public GenericArmSystemIOSparkFlex(
      int[] id,
      int currentLimitAmps,
      double restingAngle,
      boolean invert,
      boolean brake,
      double reduction,
      double kP,
      double kI,
      double kD) {
    this.reduction = reduction;
    this.restingAngle = restingAngle;
    motors = new SparkFlex[id.length];
    config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .inverted(invert)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    config.closedLoop.pid(kP, kI, kD).positionWrappingEnabled(true).outputRange(-Math.PI, Math.PI);

    for (int i = 0; i < id.length; i++) {
      motors[i] = new SparkFlex(id[i], SparkLowLevel.MotorType.kBrushless);

      if (i == 0)
        motors[i].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      else
        motors[i].configure(
            new SparkFlexConfig().apply(config).follow(motors[0]),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }
    encoder = motors[0].getEncoder();
    controller = motors[0].getClosedLoopController();
  }

  public void updateInputs(GenericArmSystemIOInputs inputs) {
    inputs.degrees = Units.rotationsToDegrees(encoder.getPosition());
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motors[0].getAppliedOutput() * motors[0].getBusVoltage();
    inputs.supplyCurrentAmps = motors[0].getOutputCurrent();
    inputs.tempCelsius = motors[0].getMotorTemperature();
  }

  @Override
  public void runToDegree(double degrees) {
    controller.setReference(degrees, ControlType.kPosition);
  }
}
