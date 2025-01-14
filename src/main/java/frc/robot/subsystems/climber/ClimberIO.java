package frc.robot.subsystems.climber;

public interface ClimberIO {
    public class ClimberIOInputs {
        public double position;
    }

    default void setPosition(double position) {};

    default void updateInputs(ClimberIOInputs io) {};
}
