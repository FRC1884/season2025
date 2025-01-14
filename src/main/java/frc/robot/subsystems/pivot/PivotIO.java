package frc.robot.subsystems.pivot;

public interface PivotIO {
    public class PivotIOInputs {
        public double degrees;
    }

    default void getDegrees(double degrees) {};

    default void updateInputs(PivotIOInputs io) {};
}
