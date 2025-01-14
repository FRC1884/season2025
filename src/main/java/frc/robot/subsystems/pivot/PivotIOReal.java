package frc.robot.subsystems.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class PivotIOReal implements PivotIO{

    private SparkMax motor = new SparkMax(PivotConstants.PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
    private PIDController pid = new PIDController(PivotConstants.kp, PivotConstants.ki, PivotConstants.kd);
    private RelativeEncoder encoder = motor.getEncoder();

    public void setDegrees(double degrees) {
        motor.setVoltage(pid.calculate(encoder.getPosition(), degrees));
    };

    public void updateInputs(PivotIOInputs io) {
        io.degrees = encoder.getPosition();
    };
}
