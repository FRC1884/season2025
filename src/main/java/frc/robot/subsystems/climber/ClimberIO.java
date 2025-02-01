package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public boolean connected = true;
    public double position;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelsius = 0.0;
  }

  default void setPosition(double position) {}
  ;

  default void updateInputs(ClimberIOInputs io) {}
  ;
}
