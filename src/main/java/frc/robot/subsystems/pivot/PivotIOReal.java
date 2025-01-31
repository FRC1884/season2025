package frc.robot.subsystems.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class PivotIOReal implements PivotIO {

  private SparkMax motor =
      new SparkMax(PivotConstants.PIVOT_ID, SparkLowLevel.MotorType.kBrushless);
  private SparkClosedLoopController controller = motor.getClosedLoopController();
  private RelativeEncoder encoder = motor.getEncoder();

  public void setDegrees(double degrees) {
    controller.setReference(degrees, SparkBase.ControlType.kMAXMotionPositionControl);
  }
  ;

  public void updateInputs(PivotIOInputs io) {
    io.degrees = encoder.getPosition();
  }
  ;
}
