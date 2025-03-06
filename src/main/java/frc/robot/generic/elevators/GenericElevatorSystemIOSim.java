package frc.robot.generic.elevators;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class GenericElevatorSystemIOSim implements GenericElevatorSystemIO {
  private ElevatorSim sim;

  private double height = 0.0;

  public GenericElevatorSystemIOSim(int numMotors, double startingAngle) {
    sim =
        new ElevatorSim(
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
    inputs.positionMeters = Units.radiansToDegrees(sim.getPositionMeters());
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
  }

  @Override
  public void runPosition(double height) {
    this.height = height;
  }
}
