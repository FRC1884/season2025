package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class ClimberIOReal implements ClimberIO{

    private SparkMax motor = new SparkMax(ClimberConstants.CLIMBER_ID, SparkLowLevel.MotorType.kBrushless);
    private PIDController pid = new PIDController(ClimberConstants.kp, ClimberConstants.ki, ClimberConstants.kd);

    public void setPosition(double position) {

    };

    public void updateInputs(ClimberIOInputs io) {

    };
}
