package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.GlobalConstants.ROBOT;
import static java.lang.Math.PI;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.GlobalConstants.Gains;
import frc.robot.GlobalConstants.RobotType;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public final class SwerveConstants {
  // Gyro
  public static enum GyroType {
    PIGEON,
    NAVX,
    ADIS,
  }

  public static final GyroType GYRO_TYPE = GyroType.PIGEON;

  /** Meters */
  public static final double TRACK_WIDTH = Units.inchesToMeters(24.5);
  /** Meters */
  public static final double WHEEL_BASE = Units.inchesToMeters(24.5);
  /** Meters */
  public static final double BUMPER_LENGTH = Units.inchesToMeters(34);
  /** Meters */
  public static final double BUMPER_WIDTH = Units.inchesToMeters(34);

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

  static final int PIGEON_ID =
      switch (ROBOT) {
        case DEVBOT -> 60;
        case COMPBOT, SIMBOT -> 60;
      };
  ;
  private static final int FRD_ID =
      switch (ROBOT) {
        case DEVBOT -> 31;
        case COMPBOT, SIMBOT -> 11;
      };
  private static final int FRR_ID =
      switch (ROBOT) {
        case DEVBOT -> 13;
        case COMPBOT, SIMBOT -> 21;
      };
  private static final int FLD_ID =
      switch (ROBOT) {
        case DEVBOT -> 32;
        case COMPBOT, SIMBOT -> 12;
      };
  private static final int FLR_ID =
      switch (ROBOT) {
        case DEVBOT -> 11;
        case COMPBOT, SIMBOT -> 22;
      };
  private static final int BRD_ID =
      switch (ROBOT) {
        case DEVBOT -> 33;
        case COMPBOT, SIMBOT -> 14;
      };
  private static final int BRR_ID =
      switch (ROBOT) {
        case DEVBOT -> 14;
        case COMPBOT, SIMBOT -> 24;
      };
  private static final int BLD_ID =
      switch (ROBOT) {
        case DEVBOT -> 34;
        case COMPBOT, SIMBOT -> 13;
      };
  private static final int BLR_ID =
      switch (ROBOT) {
        case DEVBOT -> 12;
        case COMPBOT, SIMBOT -> 23;
      };

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
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(12.6);
  /** Radians per second */
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final double WHEEL_FRICTION_COEFF = COTS.WHEELS.SLS_PRINTED_WHEELS.cof;
  /** Kilograms per square meter */
  public static final double ROBOT_INERTIA = 6.883;
  /** Kilograms */
  public static final double ROBOT_MASS = 45;

  // Drive motor configuration
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getNeoVortex(1);

  public static final double DRIVE_GEAR_RATIO = 5.08;

  static final boolean DRIVE_INVERTED = ROBOT == RobotType.DEVBOT;

  /** Amps */
  static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
  /** Amps */
  static final double DRIVE_MOTOR_MAX_TORQUE = DRIVE_GEARBOX.getTorque(DRIVE_MOTOR_CURRENT_LIMIT);

  // Drive motor PID configuration
  static final Gains DRIVE_MOTOR_GAINS =
      switch (ROBOT) {
        case DEVBOT, COMPBOT -> new Gains(0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.0, 0.0789, 0.0, 0.0);
      };

  // Drive encoder configuration
  /** Wheel radians */
  static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / DRIVE_GEAR_RATIO;
  /** Wheel radians per second */
  static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / DRIVE_GEAR_RATIO;

  // Rotator motor configuration
  public static final DCMotor TURN_GEARBOX = DCMotor.getNeo550(1);

  public static final double ROTATOR_GEAR_RATIO = 9424.0 / 203.0;
  /** Amps */
  static final int ROTATOR_MOTOR_CURRENT_LIMIT_AMPS = 20;

  static final boolean ROTATOR_INVERTED = false;

  // Rotator PID configuration
  static final Gains ROTATOR_GAINS =
      switch (ROBOT) {
        case DEVBOT, COMPBOT -> new Gains(2.0, 0.0, 0.0);
        case SIMBOT -> new Gains(8.0, 0.0, 0.0);
      };
  /** Radians */
  static final double ROTATOR_PID_MIN_INPUT = 0;
  /** Radians */
  static final double ROTATOR_PID_MAX_INPUT = 2 * Math.PI;

  // Rotator encoder configuration
  static final boolean ROTATOR_ENCODER_INVERTED = true;
  /** Radians */
  static final double ROTATOR_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
  /** Radians per second */
  static final double ROTATOR_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

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

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
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
                      WHEEL_FRICTION_COEFF, // Use the COF for Spiky Treads
                      2) // Medium Gear ratio
                  .get(),
          switch (GYRO_TYPE) {
            case PIGEON -> COTS.ofPigeon2();
            case NAVX -> COTS.ofNav2X();
            case ADIS -> COTS.ofGenericGyro();
          });
}
