package frc.robot;

import static frc.robot.GlobalConstants.ROBOT;

import frc.robot.OI.BoardOperatorMap;
import frc.robot.OI.DriverMap;
import frc.robot.OI.OperatorMap;
import frc.robot.OI.SimXboxUniversalMap;
import frc.robot.OI.XboxDriverMap;
import frc.robot.OI.XboxOperatorMap;

public final class Config {

  public static final class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = false;
    public static final boolean VISION_ENABLED = false;
    public static final boolean GAME_PIECE_VISION_ENABLED = false;
    public static final boolean FEEDER_ENABLED = false;
    public static final boolean PIVOT_ENABLED = false;
    public static final boolean SHOOTER_ENABLED = false;
    public static final boolean CLIMBER_ENABLED = false;
    public static final boolean LEDS_ENABLED = false;
    public static final boolean ELEVATOR_ENABLED = true;
  }

  public static final class Controllers {

    public static final int DRIVER_PORT = 0;

    public static final int OPERATOR_PORT = 1;
    public static final boolean JOYSTICK_OPERATOR_ENABLED = false;
    public static final boolean BOARD_OPERATOR_ENABLED = !JOYSTICK_OPERATOR_ENABLED;

    public static DriverMap getDriverController() {
      return switch (ROBOT) {
        case COMPBOT -> new XboxDriverMap(DRIVER_PORT);
        case DEVBOT -> new XboxDriverMap(DRIVER_PORT);
        case SIMBOT -> new SimXboxUniversalMap(DRIVER_PORT);
      };
    }

    public static OperatorMap getOperatorController() {
      return switch (ROBOT) {
        case COMPBOT -> JOYSTICK_OPERATOR_ENABLED
            ? new XboxOperatorMap(OPERATOR_PORT)
            : new BoardOperatorMap(OPERATOR_PORT);
        case DEVBOT -> JOYSTICK_OPERATOR_ENABLED
            ? new XboxOperatorMap(OPERATOR_PORT)
            : new BoardOperatorMap(OPERATOR_PORT);
        case SIMBOT -> new SimXboxUniversalMap(DRIVER_PORT);
      };
    }
  }
}
