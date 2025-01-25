package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOReal implements ElevatorIO {

  private SparkMax motor =
      new SparkMax(ElevatorConstants.ELEVATOR_ID, SparkLowLevel.MotorType.kBrushless);
  private PIDController pid =
      new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
  private RelativeEncoder encoder = motor.getEncoder();

  public void setPosition(double position) {
    motor.setVoltage(pid.calculate(encoder.getPosition(), position));
  }
  ;

  public void updateInputs(ElevatorIOInputs io) {
    io.position = encoder.getPosition();
  }
}
