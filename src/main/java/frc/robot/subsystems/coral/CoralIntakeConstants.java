package frc.robot.subsystems.coral;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public final class CoralIntakeConstants {
  public static final int INTAKE_CORAL_ID = 52; // TODO: Find ID for actual intake
  public static final boolean IS_FLEX = true;
  public static final DoubleSupplier currentLimit =
      new LoggedTunableNumber("CoralIntake/CurrentLimit", 15);
}
