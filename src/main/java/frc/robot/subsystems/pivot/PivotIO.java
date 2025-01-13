package frc.robot.subsystems.pivot;

public interface PivotIO {
    public class PivotIOInputs {
        private double degrees;
    }

    default void getDegrees(double degrees) {};

    default void updateInputs(PivotIOInputs io) {};
}
