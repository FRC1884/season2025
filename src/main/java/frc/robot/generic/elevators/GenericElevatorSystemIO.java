package frc.robot.generic.elevators;

import org.littletonrobotics.junction.AutoLog;

public interface GenericElevatorSystemIO {

  @AutoLog
  public static class GenericElevatorSystemIOInputs {
    public boolean[] connected = new boolean[] {};
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double encoderPosition = 0.0;
    public double goal = 0.0;
  }

  default void updateInputs(GenericElevatorSystemIOInputs inputs) {}
  /** Run elevator system to a height */
  default void setVoltage(double volts) {}
}
