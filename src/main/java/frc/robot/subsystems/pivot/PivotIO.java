package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public class PivotIOInputs {
    public double degrees;
  }

    default void setDegrees(double degrees) {};

    default void updateInputs(PivotIOInputs io) {};
}
