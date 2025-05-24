package frc.robot.subsystems.exampleClasses.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = true;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double appliedVoltage2 = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double tempEncod = 0.0;
    public double goal = 0.0;
    public boolean limitSwitch = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runVolts(double voltage) {}

  default void resetEncoder() {}
}
