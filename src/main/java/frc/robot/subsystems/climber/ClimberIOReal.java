package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class ClimberIOReal implements ClimberIO {

  private SparkMax motor =
      new SparkMax(ClimberConstants.CLIMBER_ID, SparkLowLevel.MotorType.kBrushless);
  private SparkClosedLoopController controller = motor.getClosedLoopController();
  private RelativeEncoder encoder = motor.getEncoder();

  public void setPosition(double position) {
    controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
  }
  ;

  public void updateInputs(ClimberIOInputs io) {
    io.position = encoder.getPosition();
  }
  ;
}
