package frc.robot.generic.arm;

import org.littletonrobotics.junction.AutoLog;

public interface GenericArmSystemIO {

  @AutoLog
  public static class GenericArmSystemIOInputs {
    public boolean connected = true;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double degrees = 0.0;
  }

  default void updateInputs(GenericArmSystemIOInputs inputs) {}
  /** Run arm system to an angle */
  default void runToDegree(double degrees) {}
}
