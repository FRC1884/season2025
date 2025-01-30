package frc.robot.subsystems.swerve;

import static edu.wpi.first.math.util.Units.lbsToKilograms;
import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static java.lang.Math.PI;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public final class SwerveConstants {
  public static final boolean USING_VORTEX_DRIVE = true;
  public static final DCMotor DRIVE_GEARBOX =
      USING_VORTEX_DRIVE ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1);
  /** Meters */
  public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);
  /** Meters */
  public static final double WHEEL_BASE = Units.inchesToMeters(23.5);
  /** Meters */
  public static final double BUMPER_LENGTH = Units.inchesToMeters(30);
  /** Meters */
  public static final double BUMPER_WIDTH = Units.inchesToMeters(30);

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0)
      };
  /** Meters */
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

  /**
   * Represents each module's constants on a NEO-based swerve.
   *
   * @param name of the module, for logging purposes
   * @param driveID
   * @param rotatorID
   * @param zeroRotation in radians
   */
  public record ModuleConstants(
      String name, int driveID, int rotatorID, int cancoderID, Rotation2d zeroRotation) {
    public ModuleConstants(String name, int driveID, int rotatorID, Rotation2d encoderOffset) {
      this(name, driveID, rotatorID, -1, encoderOffset);
    }
  }

  // Device CAN IDs
  static final int PIGEON_ID = 30;
  private static final int FRD_ID = 42;
  private static final int FRR_ID = 5;
  private static final int FLD_ID = 41;
  private static final int FLR_ID = 1;
  private static final int BRD_ID = 43;
  private static final int BRR_ID = 7;
  private static final int BLD_ID = 40;
  private static final int BLR_ID = 3;

  // Zeroed rotation values for each module, see setup instructions
  private static final Rotation2d FLR_ZERO = Rotation2d.fromRadians(-PI / 2);
  private static final Rotation2d FRR_ZERO = Rotation2d.fromRadians(0);
  private static final Rotation2d BLR_ZERO = Rotation2d.fromRadians(PI);
  private static final Rotation2d BRR_ZERO = Rotation2d.fromRadians(PI / 2);

  // Constants for each module. Add the CANCoder id between the rotator id and offset params
  public static final ModuleConstants FRONT_LEFT =
      new ModuleConstants("Front Left", FLD_ID, FLR_ID, FLR_ZERO);
  public static final ModuleConstants FRONT_RIGHT =
      new ModuleConstants("Front Right", FRD_ID, FRR_ID, FRR_ZERO);
  public static final ModuleConstants BACK_LEFT =
      new ModuleConstants("Back Left", BLD_ID, BLR_ID, BLR_ZERO);
  public static final ModuleConstants BACK_RIGHT =
      new ModuleConstants("Back Right", BRD_ID, BRR_ID, BRR_ZERO);

  /** Meters */
  public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);
  /** Meters per second */
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(10);
  /** Radians per second */
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final double WHEEL_FRICTION_COEFF = 1.2;
  /** Kilogram per square meter */
  public static final double ROBOT_INERTIA = 6.883;
  /** Kilograms */
  public static final double ROBOT_MASS = lbsToKilograms(110);

  // Drive motor configuration
  /** Amps */
  static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
  /** Amps */
  static final double DRIVE_MOTOR_MAX_TORQUE = DRIVE_GEARBOX.getTorque(DRIVE_MOTOR_CURRENT_LIMIT);

  // Drive motor PID configuration
  static final Gains DRIVE_GAINS = new Gains(0.0, 0.0, 0.0, 0.1);
  static final Gains DRIVE_SIM_GAINS = new Gains(0.05, 0.0, 0.0, 0.085);

  // Drive encoder configuration
  // this and below for choreo and pathfinding
  public static final double DRIVE_GEAR_RATIO = 5.08;
  /** Radians per second per second */
  public static final double MAX_ANGULAR_ACCELERATION =
      4 * (DRIVE_GEAR_RATIO * DRIVE_MOTOR_MAX_TORQUE) / WHEEL_RADIUS * DRIVE_BASE_RADIUS / 15.0;
  /** Meters per second per second */
  public static final double MAX_LINEAR_ACCELERATION =
      4 * (DRIVE_GEAR_RATIO * DRIVE_MOTOR_MAX_TORQUE) / WHEEL_RADIUS / ROBOT_MASS;
  /** Wheel radians */
  static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_GEAR_RATIO;
  /** Wheel radians per second */
  static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / DRIVE_GEAR_RATIO;

  // Rotator motor configuration
  /** Amps */
  static final int ROTATOR_MOTOR_CURRENT_LIMIT_AMPS = 20;

  static final boolean ROTATOR_INVERTED = false;

  // Rotator PID configuration
  static final Gains ROTATOR_GAINS = new Gains(2.0, 0.0);
  static final Gains ROTATOR_SIM_GAINS = new Gains(8.0, 0.0);
  /** Radians */
  static final double TURN_PID_MIN_INPUT = 0;
  /** Radians */
  static final double TURN_PID_MAX_INPUT = 2 * Math.PI;

  // Rotator encoder configuration
  static final boolean TURN_ENCODER_INVERTED = true;
  /** Radians */
  static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
  /** Radians per second */
  static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Drivetrain PID
  public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(10, 0, 0);
  public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(100, 0, 0);

  // PathPlanner configuration
  public static final RobotConfig PATHPLANNER_CONFIG =
      new RobotConfig(
          ROBOT_MASS,
          ROBOT_INERTIA,
          new ModuleConfig(
              WHEEL_RADIUS,
              MAX_LINEAR_SPEED,
              WHEEL_FRICTION_COEFF,
              DRIVE_GEARBOX.withReduction(DRIVE_GEAR_RATIO),
              DRIVE_MOTOR_CURRENT_LIMIT,
              1),
          MODULE_TRANSLATIONS);

  public static final double TURN_GEAR_RATIO = 9424.0 / 203.0;

  public static final DCMotor TURN_GEARBOX = DCMotor.getNeo550(1);
  public static final DriveTrainSimulationConfig mapleSimConfig =
      new DriveTrainSimulationConfig(
          Kilograms.of(ROBOT_MASS),
          Meters.of(BUMPER_LENGTH),
          Meters.of(BUMPER_WIDTH),
          Meters.of(WHEEL_BASE),
          Meters.of(TRACK_WIDTH),
          () ->
              COTS.ofMAXSwerve(
                      DRIVE_GEARBOX, // Drive motor is a Neo Vortex
                      TURN_GEARBOX, // Steer motor is a Neo 550
                      WHEEL_FRICTION_COEFF, // Use the COF for Colson Wheels
                      2) // Medium Gear ratio
                  .get(),
          COTS.ofNav2X());

  // Gyro
  static enum GyroType {
    PIGEON,
    NAVX,
    ADIS,
  }

  static final GyroType GYRO_TYPE = GyroType.NAVX;
}
