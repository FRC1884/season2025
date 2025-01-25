package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public class ElevatorIOInputs {
        public double position;
    }

    default void setPosition(double position) {};

    default void updateInputs(frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs io) {};
}