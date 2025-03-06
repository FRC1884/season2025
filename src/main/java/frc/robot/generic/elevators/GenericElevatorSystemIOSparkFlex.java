package frc.robot.generic.elevators;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class GenericElevatorSystemIOSparkFlex implements GenericElevatorSystemIO {
  private final SparkFlex[] motors;
  private final RelativeEncoder encoder;
  private SparkBaseConfig config;
  private SparkClosedLoopController controller;
  private DoubleSupplier kp, ki, kd;
  private boolean[] inverted;
  private boolean firstRun;
  private boolean limitSwitch = false;
  private ProfiledPIDController pidController;
  private ElevatorFeedforward feedforward;
  private SparkFlex leader;

  public GenericElevatorSystemIOSparkFlex(
      int[] id,
      boolean[] inverted,
      int currentLimitAmps,
      double restingAngle,
      boolean brake,
      double reduction,
      DoubleSupplier kP,
      DoubleSupplier kI,
      DoubleSupplier kD) {
    this.kp = kP;
    this.ki = kI;
    this.kd = kD;
    this.inverted = inverted;

    motors = new SparkFlex[id.length];
    config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    pidController = new ProfiledPIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble(), new TrapezoidProfile.Constraints(6000, 6000));

    leader = motors[0];

    leader.configure(
      config.inverted(inverted[0]),
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);

    for (int i = 1; i < id.length; i++) {
      motors[i] = new SparkFlex(id[i], SparkLowLevel.MotorType.kBrushless);
        motors[i].configure(
            (new SparkFlexConfig().apply(config).follow(motors[0], inverted[i])),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
    }
    encoder = leader.getEncoder();
    pidController.setTolerance(0.5);
  }

  @Override
  public void updateInputs(GenericElevatorSystemIOInputs inputs) {
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    inputs.appliedVoltage = leader.getAppliedOutput() * motors[0].getBusVoltage();
    inputs.supplyCurrentAmps = leader.getOutputCurrent();
    inputs.tempCelsius = leader.getMotorTemperature();
    inputs.goal = pidController.getGoal().position;
    inputs.limitSwitch = limitSwitch;

    Logger.recordOutput("Elevator/AtGoal", pidController.atGoal());
  }

  @Override
  public void runPosition(double position) {
    pidController.setGoal(position);
    double pidVolts = pidController.calculate(encoder.getPosition());
    double ffVolts = feedforward.calculate(pidController.getSetpoint().velocity);
    leader.setVoltage(MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0));
  }

  @Override
  public void resetEncoder(boolean limitSwitch) {
    // encoder.setPosition(0.0);
    this.limitSwitch = limitSwitch;
  }
}
