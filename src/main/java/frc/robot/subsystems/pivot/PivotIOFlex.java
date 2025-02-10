package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.generic.arm.GenericArmSystemIOSparkFlex;
import java.util.Map;

public class PivotIOFlex extends GenericArmSystemIOSparkFlex implements PivotIO {
  private static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config
        .smartCurrentLimit(40)
        .closedLoop
        .pidf(GAINS.kP(), 0.0, GAINS.kD(), GAINS.kV())
        .maxMotion
        .maxAcceleration(MAX_VELOCITY)
        .maxVelocity(MAX_ACCELERATION)
        .allowedClosedLoopError(POSITION_TOLERANCE);
  }

  public PivotIOFlex() {
    super(1.0, config, Map.of(PIVOT_ID, false));
  }
}
