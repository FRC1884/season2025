package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public class PivotIOInputs {
    public boolean connected = true;
    public double degrees;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelsius = 0.0;
  }

  default void setDegrees(double degrees) {}
  ;

  default void updateInputs(PivotIOInputs io) {}
  ;
}
