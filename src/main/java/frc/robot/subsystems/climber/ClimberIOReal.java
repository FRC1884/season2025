package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.pivot.PivotConstants;

import static frc.robot.subsystems.pivot.PivotConstants.*;

public class ClimberIOReal implements ClimberIO {

  private SparkMax motor;
  private SparkClosedLoopController controller;
  private RelativeEncoder encoder;
  private SparkMaxConfig config;

  public ClimberIOReal() {
    motor = new SparkMax(PivotConstants.PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.closedLoop
            .p(kp)
            .i(ki)
            .d(kd);
    config.closedLoop.maxMotion
            .maxVelocity(6000)
            .maxAcceleration(6000)
            .allowedClosedLoopError(0.02);
    motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    controller = motor.getClosedLoopController();
    encoder = motor.getEncoder();
  }

  public void setPosition(double position) {
    controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
  }
  ;

  public void updateInputs(ClimberIOInputs io) {
    io.position = encoder.getPosition();
    io.velocityRadsPerSec = Units.RadiansPerSecond.convertFrom(encoder.getVelocity(), Units.RotationsPerSecond);
    io.appliedVoltage = motor.getAppliedOutput() + motor.getBusVoltage();
    io.tempCelsius = motor.getMotorTemperature();
  }
  ;
}
