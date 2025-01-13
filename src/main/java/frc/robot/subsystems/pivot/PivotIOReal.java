package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class PivotIOReal implements PivotIO{

    private SparkMax motor = new SparkMax(PivotConstants.PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
    private PIDController pid = new PIDController(PivotConstants.kp, PivotConstants.ki, PivotConstants.kd);

    public void setPosition(double position) {

    };

    public void updateInputs(PivotIOInputs io) {

    };
}
