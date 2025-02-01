package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;

public class FeederIOReal implements FeederIO {

    private SparkMaxConfig config;
    private SparkClosedLoopController controller;
    private SparkMax motor;
    private RelativeEncoder encoder;

    public FeederIOReal(){
        motor = new SparkMax(FeederConstants.FEEDER_ID, SparkLowLevel.MotorType.kBrushless);
        // made motor
        config = new SparkMaxConfig();
        config.closedLoop.p(FeederConstants.kp).i(FeederConstants.ki).d(FeederConstants.kd);

        controller = motor.getClosedLoopController();
        encoder = motor.getEncoder();
    }

    @Override
    public void changeVoltage(double voltage){
        motor.setVoltage(voltage);
    }
    @Override
    public void setSpeed(double speed) {
        controller.setReference(speed, SparkBase.ControlType.kVelocity);

    }
    @Override
    public void updateInputs(feederIOInputs IO) {
        IO.speed = encoder.getVelocity();
    }



}
