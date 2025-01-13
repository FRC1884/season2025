package frc.robot.subsystems.climber;

public interface ClimberIO {
    public class ClimberIOInputs {
        private double position;
    }

    default void setPosition(double position) {};

    default void updateInputs(ClimberIOInputs io) {};
}
