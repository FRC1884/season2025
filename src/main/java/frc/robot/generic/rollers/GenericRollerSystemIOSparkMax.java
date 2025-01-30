package frc.robot.generic.rollers;

import static com.revrobotics.spark.SparkBase.PersistMode;
import static com.revrobotics.spark.SparkBase.ResetMode;
import static com.revrobotics.spark.SparkLowLevel.MotorType;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public abstract class GenericRollerSystemIOSparkMax implements GenericRollerSystemIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;

  private final double reduction;

  public GenericRollerSystemIOSparkMax(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    sparkMax = new SparkMax(id, MotorType.kBrushless);

    var config =
        new SparkMaxConfig()
            .smartCurrentLimit(currentLimitAmps)
            .inverted(invert)
            .idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

    sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = sparkMax.getEncoder();
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.supplyCurrentAmps = sparkMax.getOutputCurrent();
    inputs.tempCelsius = sparkMax.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    sparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    sparkMax.stopMotor();
  }
}
