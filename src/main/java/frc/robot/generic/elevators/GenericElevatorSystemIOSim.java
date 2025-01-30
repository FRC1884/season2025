package frc.robot.generic.elevators;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GenericElevatorSystemIOSim implements GenericElevatorSystemIO {
  private SingleJointedArmSim sim;

  private double height = 0.0;

  public GenericElevatorSystemIOSim(int numMotors, double startingAngle) {
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
  public void updateInputs(GenericElevatorSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runPosition(height);
    }

    sim.update(0.02);
    inputs.positionMeters = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityMetersPerSec = sim.getVelocityRadPerSec();
  }

  @Override
  public void runPosition(double height) {
    this.height = height;
  }
}
