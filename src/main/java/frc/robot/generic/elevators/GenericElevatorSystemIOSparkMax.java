package frc.robot.generic.elevators;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class GenericElevatorSystemIOSparkMax implements GenericElevatorSystemIO {
    private final SparkMax[] motors;
    private final RelativeEncoder encoder;
    private final PIDController pidController;
    private final Object restingPosition;
    private final double reduction;

    public GenericElevatorSystemIOSparkMax(
            int[] id,
            int currentLimitAmps,
            double restingPosition,
            boolean invert,
            boolean brake,
            double reduction,
            double kP,
            double kI,
            double kD) {
        this.reduction = reduction;
        this.restingPosition = restingPosition;
        motors = new SparkMax[id.length];
        pidController = new PIDController(kP, kI, kD);
        var config =
                new SparkMaxConfig()
                        .smartCurrentLimit(currentLimitAmps)
                        .inverted(invert)
                        .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

        for (int i = 0; i < id.length; i++) {
            motors[i] = new SparkMax(id[i], SparkLowLevel.MotorType.kBrushless);

            if (i == 0)
                motors[i].configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
            else
                motors[i].configure(
                        new SparkMaxConfig().follow(motors[0]),
                        SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
        }

        encoder = motors[0].getEncoder();
    }

    public void updateInputs(GenericElevatorSystemIO.GenericElevatorSystemIOInputs inputs) {
        inputs.positionMeters = Units.rotationsToRadians(encoder.getPosition()) / reduction;
        inputs.velocityMetersPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
        inputs.appliedVoltage = motors[0].getAppliedOutput() * motors[0].getBusVoltage();
        inputs.supplyCurrentAmps = motors[0].getOutputCurrent();
        inputs.tempCelsius = motors[0].getMotorTemperature();
    }

    @Override
    public void runToPosition(double position) {
        motors[0].setVoltage(pidController.calculate(encoder.getPosition(), position));
    }
}

}
