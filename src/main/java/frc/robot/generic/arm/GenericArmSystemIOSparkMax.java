package frc.robot.generic.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

import static frc.robot.GlobalConstants.TUNING_MODE;

public class GenericArmSystemIOSparkMax implements GenericArmSystemIO {
  private final SparkMax[] motors;
  private final AbsoluteEncoder encoder;
  private SparkBaseConfig config;
  private SparkClosedLoopController controller;
  private double goal;
  private DoubleSupplier kp, ki, kd;
  private boolean[] inverted;
  private double forwardLimit;
  private double reverseLimit;

  public GenericArmSystemIOSparkMax(
      int[] id,
      boolean[] inverted,
      int currentLimitAmps,
      boolean brake,
      double forwardLimit,
      double reverseLimit,
      DoubleSupplier kP,
      DoubleSupplier kI,
      DoubleSupplier kD,
      boolean absInverted) {
    this.kp = kP;
    this.ki = kI;
    this.kd = kD;
    this.inverted = inverted;
    this.reverseLimit = reverseLimit;
    this.forwardLimit = forwardLimit;
    motors = new SparkMax[id.length];
    config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    config.absoluteEncoder.inverted(absInverted);
    config.closedLoop.pid(kP.getAsDouble(), 0, 0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    config.softLimit.forwardSoftLimit(forwardLimit).reverseSoftLimit(reverseLimit).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

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
    encoder = motors[0].getAbsoluteEncoder();
    controller = motors[0].getClosedLoopController();
  }

  public void updateInputs(GenericArmSystemIOInputs inputs) {
    if (motors != null) {
      inputs.degrees = Units.rotationsToDegrees(encoder.getPosition());
      inputs.rotations = encoder.getPosition();
      inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
      inputs.appliedVoltage = motors[0].getAppliedOutput() * motors[0].getBusVoltage();
      inputs.supplyCurrentAmps = motors[0].getOutputCurrent();
      inputs.tempCelsius = motors[0].getMotorTemperature();
      inputs.goal = goal;
      if (encoder.getPosition() < reverseLimit || encoder.getPosition() > forwardLimit)
        motors[0].stopMotor();
    }
  }

  @Override
  public void runToDegree(double angle) {
     if (TUNING_MODE) {
       config.closedLoop.pid(kp.getAsDouble(), ki.getAsDouble(), kd.getAsDouble());
       motors[0].configure(
           config.inverted(inverted[0]),
           ResetMode.kNoResetSafeParameters,
           PersistMode.kNoPersistParameters);
     }
    controller.setReference(angle, ControlType.kPosition);
    goal = angle;
  }
}
