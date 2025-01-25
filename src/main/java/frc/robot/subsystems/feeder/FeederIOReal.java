package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class FeederIOReal implements FeederIO {

    private PIDController pid = new PIDController(FeederConstants.kp, FeederConstants.ki, FeederConstants.kd);
    private SparkMax motor = new SparkMax(FeederConstants.FEEDER_ID, SparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();

    @Override
    public void changeVoltage(double voltage){
        motor.setVoltage(voltage);
    }
    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }
    @Override
    public void updateInputs(feederIOInputs IO) {
        IO.speed = encoder.getVelocity();
    }



}
