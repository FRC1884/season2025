package frc.robot.subsystems.exampleClasses.roller;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class IntakeConstants {
  public static final int INTAKE_ALGAE_ID = 62; // TODO: Find ID for actual intake
  public static final boolean isFlex = true;
  public static final DoubleSupplier currentLimit =
      new LoggedTunableNumber("CoralIntake/CurrentLimit", 15);
}
