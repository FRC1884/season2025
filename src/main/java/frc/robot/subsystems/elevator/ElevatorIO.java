package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public double position;
  }

  default void setPosition(double position) {}
  ;

  default void updateInputs(frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs io) {}
  ;
}
