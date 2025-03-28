package frc.robot.generic.elevators;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLog;

public interface GenericElevatorSystemIO {

  @AutoLog
  public static class GenericElevatorSystemIOInputs {
    public boolean connected = true;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double appliedVoltageTwo = 0.0;
    public double supplyCurrentAmpstwo = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double tempEncod = 0.0;
    public double goal = 0.0;
    public boolean limitSwitch = false;
    public boolean atGoal = false;
  }

  default void updateInputs(GenericElevatorSystemIOInputs inputs) {}
  /** Run elevator system to a height */
  default void runPosition(double height) {}
  /** update first run */
  default void updatePID() {}
  /** Stop elevator system */
  default Command resetEncoderSequence() {
    return Commands.none();
  }
}
