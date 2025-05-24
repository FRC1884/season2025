package frc.robot.subsystems.exampleClasses.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {
  private SingleJointedArmSim sim;

  private double voltage = 0.0;

  public ClimberIOSim() {
    sim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(1),
            (10 / Units.metersToInches(0.012) / 0.5),
            1,
            0.3126232,
            0,
            Units.degreesToRadians(110),
            true,
            50);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    sim.update(0.02);
    inputs.positionMeters = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityMetersPerSec = sim.getVelocityRadPerSec();
  }

  @Override
  public void runVolts(double volts) {
    this.voltage = volts;
    sim.setInputVoltage(volts);
  }
}
