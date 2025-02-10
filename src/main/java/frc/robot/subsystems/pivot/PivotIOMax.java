package frc.robot.subsystems.pivot;

import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.generic.arm.GenericArmSystemIOSparkFlex;
import java.util.Map;

public class PivotIOMax extends GenericArmSystemIOSparkFlex implements PivotIO {
  public PivotIOMax() {
    super(1.0, new SparkMaxConfig(), Map.of(40, false));
  }
}
