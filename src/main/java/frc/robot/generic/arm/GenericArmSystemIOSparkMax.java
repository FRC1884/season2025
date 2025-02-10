package frc.robot.generic.arm;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import java.util.*;

public class GenericArmSystemIOSparkMax implements GenericArmSystemIO {
  private final List<SparkMax> motors = new ArrayList<>();
  private SparkMax leader;
  private RelativeEncoder encoder;
  private SparkClosedLoopController controller;
  private double restingAngle;

  public GenericArmSystemIOSparkMax(
      double restingAngle, SparkMaxConfig config, Map<Integer, Boolean> idToInverted) {
    this.restingAngle = restingAngle;

    for (Integer id : idToInverted.keySet()) {
      motors.add(new SparkMax(id, MotorType.kBrushless));
    }

    if (motors.size() > 0) {
      leader = motors.get(0);
      leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      encoder = leader.getEncoder();
      controller = leader.getClosedLoopController();
    }

    if (motors.size() > 1)
      motors.stream()
          .skip(1)
          .forEach(
              follower ->
                  follower.configure(
                      new SparkMaxConfig().apply(config).follow(leader, true),
                      ResetMode.kResetSafeParameters,
                      PersistMode.kPersistParameters));
  }

  public void updateInputs(GenericArmSystemIOInputs inputs) {
    for (int i = 0; i < motors.size(); i++)
      inputs.connected[i] = motors.get(i).clearFaults() == REVLibError.kOk;
    if (leader != null) {
      inputs.degrees = Units.rotationsToDegrees(encoder.getPosition());
      inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
      inputs.appliedVoltage = leader.getAppliedOutput() * leader.getBusVoltage();
      inputs.supplyCurrentAmps = leader.getOutputCurrent();
      inputs.tempCelsius = leader.getMotorTemperature();
    }
  }

  @Override
  public void runToDegree(double degrees) {
    if (leader != null)
      controller.setReference(degrees, SparkBase.ControlType.kMAXMotionPositionControl);
  }
}
