package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import java.util.function.DoubleSupplier;

public class FeederSubsystems extends SubsystemBase {


    @Getter
    public enum FeederState {
        IDLING(() -> 0.0),
        ON(() -> 12.0),
        REVERSE(() -> -12.0);

        private final DoubleSupplier speed;

        FeederState(DoubleSupplier speed) {
            this.speed = speed;
        }

    }

    private FeederState feederState = FeederState.IDLING;
}