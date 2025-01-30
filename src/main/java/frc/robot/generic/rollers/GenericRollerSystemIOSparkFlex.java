package frc.robot.generic.rollers;

import static com.revrobotics.spark.SparkBase.PersistMode;
import static com.revrobotics.spark.SparkBase.ResetMode;
import static com.revrobotics.spark.SparkLowLevel.MotorType;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

public abstract class GenericRollerSystemIOSparkFlex implements GenericRollerSystemIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;

  private final double reduction;

  public GenericRollerSystemIOSparkFlex(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    motor = new SparkFlex(id, MotorType.kBrushless);

    var config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .inverted(invert)
            .idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
