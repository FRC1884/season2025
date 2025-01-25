package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double position;
  }

    default void setPosition(double position) {};

    default void updateInputs(ClimberIOInputs io) {};
}
