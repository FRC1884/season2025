package frc.robot.generic.arms;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GenericArmSystemIOSim implements GenericArmSystemIO {
  private SingleJointedArmSim sim;

  private double degree = 0.0;

  public GenericArmSystemIOSim(int numMotors, double startingAngle) {
    sim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(numMotors),
            (10 / Units.metersToInches(0.012) / 0.5),
            1,
            0.3126232,
            0,
            Units.degreesToRadians(110),
            true,
            startingAngle);
  }

  @Override
  public void updateInputs(GenericArmSystemIOInputs inputs) {
    sim.update(0.02);
    inputs.encoderPosition = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocity = sim.getVelocityRadPerSec();
    inputs.encoderPosition = sim.getAngleRads();
  }

  @Override
  public void setVoltage(double degrees) {
    this.degree = degrees;
  }
}
