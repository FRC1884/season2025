package frc.robot.generic.elevators;

import org.littletonrobotics.junction.AutoLog;

public interface GenericElevatorSystemIO {

  @AutoLog
  public static class GenericElevatorSystemIOInputs {
    public boolean connected = true;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(GenericElevatorSystemIOInputs inputs) {}
  /** Run elevator system to a height */
  default void runPosition(double height) {}
  /** Stop elevator system */
  default void stop() {}
}
