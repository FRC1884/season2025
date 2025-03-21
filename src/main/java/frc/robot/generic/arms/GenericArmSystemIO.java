package frc.robot.generic.arms;

import org.littletonrobotics.junction.AutoLog;

public interface GenericArmSystemIO {

  @AutoLog
  public static class GenericArmSystemIOInputs {
    public boolean[] connected = new boolean[] {};
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double encoderPosition = 0.0;
    public double goal = 0.0;
  }

  default void updateInputs(GenericArmSystemIOInputs inputs) {}
  /** Run arm system to an angle */
  default void setVoltage(double volts) {}
}
