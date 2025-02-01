package frc.robot.subsystems.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;

import static frc.robot.subsystems.pivot.PivotConstants.*;

public class PivotIOReal implements PivotIO {

  private SparkMax motor;
  private SparkClosedLoopController controller;
  private RelativeEncoder encoder;
  private SparkMaxConfig config;

  public PivotIOReal() {
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

  public void setDegrees(double degrees) {
    controller.setReference(degrees, SparkBase.ControlType.kMAXMotionPositionControl);
  }
  ;

  public void updateInputs(PivotIOInputs io) {
    io.degrees = encoder.getPosition();
    io.velocityRadsPerSec = Units.RadiansPerSecond.convertFrom(encoder.getVelocity(), Units.RotationsPerSecond);
    io.appliedVoltage = motor.getAppliedOutput() + motor.getBusVoltage();
    io.tempCelsius = motor.getMotorTemperature();
  }
  ;
}
