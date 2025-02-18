package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOMax implements ClimberIO {
  private final SparkMax[] motors;
  private RelativeEncoder encoder;
  private SparkBaseConfig config;
  private DigitalInput limitSwitch;
  private boolean[] inverted;

  public ClimberIOMax(int[] id, boolean[] inverted, int currentLimitAmps, boolean brake) {
    this.inverted = inverted;
    this.limitSwitch = new DigitalInput(ClimberConstants.DIOPort);
    motors = new SparkMax[id.length];
    config =
        new SparkMaxConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

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
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    inputs.appliedVoltage = motors[0].getAppliedOutput() * motors[0].getBusVoltage();
    inputs.supplyCurrentAmps = motors[0].getOutputCurrent();
    inputs.tempCelsius = motors[0].getMotorTemperature();
    inputs.limitSwitch = limitSwitch.get();
  }

  @Override
  public void runVolts(double volts) {
    if (limitSwitch.get()) {
      motors[0].stopMotor();
      motors[1].stopMotor();
    } else motors[0].setVoltage(volts);
  }
}
