package frc.robot.generic.rollers;

import static frc.robot.Config.Subsystems.FEEDER_ENABLED;
import static frc.robot.GlobalConstants.MODE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import lombok.Getter;

public class Rollers extends SubsystemBase {
  @Getter
  private final FeederSubsystem feeder =
      Config.Subsystems.FEEDER_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new FeederSubsystem("Feeder", new FeederIOReal())
              : new FeederSubsystem(
                  "Feeder Sim", new FeederIOSim(new DCMotor(1, 1, 1, 1, 1, 1), 0, 0)))
          : null;

  @Override
  public void periodic() {
    if (FEEDER_ENABLED) feeder.periodic();
  }
}
