package frc.robot.generic.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
    if (DriverStation.isDisabled()) {
      runToDegree(degree);
    }

    sim.update(0.02);
    inputs.degrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
  }

  @Override
  public void runToDegree(double degrees) {
    this.degree = degrees;
  }
}
