package frc.robot.subsystems.feeder;

public interface FeederIO {
    public class feederIOInputs {
        public double speed;
    }
    default void setSpeed(double speed) {}

    default void updateInputs(feederIOInputs IO) {}

    default void changeVoltage(double voltage){}

}
