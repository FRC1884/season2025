package frc.robot.generic.arms;

import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class GenericArmSystemIOSparkMax implements GenericArmSystemIO {
  private final SparkMax[] motors;
  private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;
  private SparkBaseConfig config;
  private final SparkMax leader;

  public GenericArmSystemIOSparkMax(
      int[] ids, int currentLimitAmps, boolean brake, double forwardLimit, double reverseLimit) {

    motors = new SparkMax[ids.length];
    config =
        new SparkFlexConfig().smartCurrentLimit(currentLimitAmps).idleMode(brake ? kBrake : kCoast);
    config.softLimit.forwardSoftLimit(forwardLimit).reverseSoftLimit(reverseLimit);

    leader = motors[0] = new SparkMax(ids[0], SparkLowLevel.MotorType.kBrushless);
    leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (ids.length > 1) {
      for (int i = 1; i < ids.length; i++) {
        motors[i] = new SparkMax(ids[i], SparkLowLevel.MotorType.kBrushless);
        motors[i].configure(
            new SparkFlexConfig().apply(config).follow(leader),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      }
    }
    absoluteEncoder = leader.getAbsoluteEncoder();
    relativeEncoder = leader.getEncoder();
  }

  public void updateInputs(GenericArmSystemIOInputs inputs) {
    if (motors != null) {
      inputs.encoderPosition =
          (absoluteEncoder != null) ? absoluteEncoder.getPosition() : relativeEncoder.getPosition();
      inputs.velocity =
          (absoluteEncoder != null) ? absoluteEncoder.getVelocity() : relativeEncoder.getVelocity();
      inputs.appliedVoltage = leader.getAppliedOutput() * leader.getBusVoltage();
      inputs.supplyCurrentAmps = leader.getOutputCurrent();
      inputs.tempCelsius = leader.getMotorTemperature();
    }
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  protected void invert(int id) {
    motors[id].configure(
        new SparkFlexConfig().inverted(true),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
