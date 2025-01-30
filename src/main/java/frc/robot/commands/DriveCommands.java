// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.RotationalAllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  // driving
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  // characterization
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(-omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static void chassisSpeedDrive(SwerveSubsystem drive, ChassisSpeeds speeds) {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getRotation()));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            SwerveConstants.ROTATION_CONSTANTS.kP,
            0.0,
            SwerveConstants.ROTATION_CONSTANTS.kD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  // use PID to align to a target
  public static Command chasePoseRobotRelativeCommand(
      SwerveSubsystem drive, Supplier<Transform2d> targetOffset) {
    TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    // TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);

    ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0, X_CONSTRAINTS);
    ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0, Y_CONSTRAINTS);
    PIDController omegaPID = new PIDController(0.01, 0, 0);

    xController.setTolerance(0.10);
    yController.setTolerance(0.03);
    omegaPID.setTolerance(1.5);
    omegaPID.enableContinuousInput(-180, 180);

    return new DeferredCommand(
        () ->
            new FunctionalCommand(
                () -> {
                  // Init
                },
                () -> {
                  double ySpeed = yController.calculate(0, targetOffset.get().getY());
                  double xSpeed = xController.calculate(0, targetOffset.get().getX());
                  double omegaSpeed =
                      omegaPID.calculate(0, targetOffset.get().getRotation().getDegrees());

                  DriveCommands.joystickDrive(drive, () -> xSpeed, () -> ySpeed, () -> omegaSpeed);
                },
                interrupted -> {
                  DriveCommands.chassisSpeedDrive(drive, new ChassisSpeeds());
                  omegaPID.close();
                  System.out.println("aligned now");
                },
                () -> {
                  return omegaPID.atSetpoint() && xController.atGoal() && yController.atGoal();
                },
                drive),
        Set.of(drive));
  }

  /**
   * Command to align the robot to a target with robot relative driving with an override of the
   * robot's x direction using the driver's y axis
   */
  public static Command chasePoseRobotRelativeCommandYOverride(
      SwerveSubsystem drive, Supplier<Transform2d> targetOffset, DoubleSupplier yDriver) {
    TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    // TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);

    ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0, X_CONSTRAINTS);
    ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0, Y_CONSTRAINTS);
    PIDController omegaPID = new PIDController(0.01, 0, 0);

    xController.setTolerance(0.10);
    yController.setTolerance(0.03);
    omegaPID.setTolerance(1.5);
    omegaPID.enableContinuousInput(-180, 180);

    return new DeferredCommand(
        () ->
            new FunctionalCommand(
                () -> {
                  // Init
                },
                () -> {
                  double driverInputFactor = 2;
                  double ySpeed = -yDriver.getAsDouble() * driverInputFactor;
                  double xSpeed = xController.calculate(0, targetOffset.get().getX());
                  double omegaSpeed =
                      omegaPID.calculate(0, targetOffset.get().getRotation().getRadians());

                  DriveCommands.joystickDrive(drive, () -> xSpeed, () -> ySpeed, () -> omegaSpeed);
                },
                interrupted -> {
                  DriveCommands.chassisSpeedDrive(drive, new ChassisSpeeds());
                  omegaPID.close();
                  System.out.println("aligned now");
                },
                () -> {
                  return omegaPID.atSetpoint() && xController.atGoal() && yController.atGoal();
                },
                drive),
        Set.of(drive));
  }

  /** Updates the path to override for the coral offset */
  public static Command overridePathplannerCoralOffset(DoubleSupplier offset) {
    return Commands.run(
        () ->
            PPHolonomicDriveController.overrideYFeedback(
                () -> {
                  // Calculate feedback from your custom PID controller
                  return offset.getAsDouble();
                }));
  }

  /** clears all x and y overrides */
  public static Command clearXYOverrides() {
    return Commands.run(() -> PPHolonomicDriveController.clearXYFeedbackOverride());
  }

  /** code for reef alignment */

  // align to target face
  public static Command alignToReefCommnd(SwerveSubsystem drive) {
    // find the coordinates of the selected face
    Pose2d targetFace;
    if (targetReefFace == 1) targetFace = GlobalConstants.FieldMap.Coordinates.REEF1.getPose();
    if (targetReefFace == 2) targetFace = GlobalConstants.FieldMap.Coordinates.REEF2.getPose();
    if (targetReefFace == 3) targetFace = GlobalConstants.FieldMap.Coordinates.REEF3.getPose();
    if (targetReefFace == 4) targetFace = GlobalConstants.FieldMap.Coordinates.REEF4.getPose();
    if (targetReefFace == 5) targetFace = GlobalConstants.FieldMap.Coordinates.REEF5.getPose();
    if (targetReefFace == 6) targetFace = GlobalConstants.FieldMap.Coordinates.REEF6.getPose();
    else targetFace = findClosestReefFace(drive);

    double xOffset = Units.feetToMeters(-GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET / 12);
    double yOffset =
        Units.feetToMeters(GlobalConstants.AlignOffsets.REEF_TO_BRANCH_OFFSET / 12)
            * (leftAlign ? 1 : -1);
    Rotation2d rotation = targetFace.getRotation();
    Transform2d branchTransform =
        new Transform2d(new Translation2d(xOffset, yOffset).rotateBy(rotation), new Rotation2d());
    Pose2d target = targetFace.transformBy(branchTransform);

    PathConstraints constraints =
        new PathConstraints(
            SwerveConstants.MAX_LINEAR_SPEED,
            SwerveConstants.MAX_LINEAR_ACCELERATION,
            SwerveConstants.MAX_ANGULAR_SPEED,
            SwerveConstants.MAX_ANGULAR_ACCELERATION);

    double endVelocity = 0.0;

    if (targetReefFace > 0) {
      // reset reef face
      targetReefFace = 0;
      return AutoBuilder.pathfindToPose(target, constraints, endVelocity);
    }

    Transform2d targetOffset = target.minus(drive.getPose());

    return chasePoseRobotRelativeCommand(drive, () -> targetOffset);
  }

  /** helper methods for alignment */

  // store the reef face target
  private static int targetReefFace = 0;

  private static boolean leftAlign = false;

  public static void setTargetReefFace(int targetReefFace) {
    DriveCommands.targetReefFace = targetReefFace;
  }

  public static void setLeftAlign(boolean leftAlign) {
    DriveCommands.leftAlign = leftAlign;
  }

  // returns the nearest face of the reef
  private static Pose2d findClosestReefFace(SwerveSubsystem drive) {
    double reef1 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF1.getPose().getTranslation());
    double reef2 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF2.getPose().getTranslation());
    double reef3 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF3.getPose().getTranslation());
    double reef4 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF4.getPose().getTranslation());
    double reef5 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF5.getPose().getTranslation());
    double reef6 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF6.getPose().getTranslation());

    double closestFace =
        Math.min(Math.min(Math.min(reef1, reef2), Math.min(reef3, reef4)), Math.min(reef5, reef6));
    if (reef1 == closestFace) return GlobalConstants.FieldMap.Coordinates.REEF1.getPose();
    if (reef2 == closestFace) return GlobalConstants.FieldMap.Coordinates.REEF2.getPose();
    if (reef3 == closestFace) return GlobalConstants.FieldMap.Coordinates.REEF3.getPose();
    if (reef4 == closestFace) return GlobalConstants.FieldMap.Coordinates.REEF4.getPose();
    if (reef5 == closestFace) return GlobalConstants.FieldMap.Coordinates.REEF5.getPose();
    return GlobalConstants.FieldMap.Coordinates.REEF6.getPose();
  }
  /**
   * Command to align to the nearest coral station
   *
   * @param drive
   * @return
   */
  public static Command alignToNearestCoralStationCommand(
      SwerveSubsystem drive, DoubleSupplier yDriver) {
    DoubleSupplier driver =
        () -> yDriver.getAsDouble() * (shouldFlipDriverOverride(drive) ? -1 : 1);
    Supplier<Transform2d> targetOffset =
        () -> findClosestCoralStation(drive).minus(drive.getPose());

    return chasePoseRobotRelativeCommandYOverride(drive, targetOffset, driver);
  }

  /** helper methods for alignment */

  // returns the coordinates of the nearest coral station
  private static Pose2d findClosestCoralStation(SwerveSubsystem drive) {
    if (RotationalAllianceFlipUtil.apply(drive.getPose()).getTranslation().getY()
        < GlobalConstants.FieldMap.FIELD_WIDTH_METERS / 2) {
      return GlobalConstants.FieldMap.Coordinates.RIGHT_CORAL_STATION.getPose();
    } else {
      return GlobalConstants.FieldMap.Coordinates.LEFT_CORAL_STATION.getPose();
    }
  }

  // returns whether the driver y-input for aligning should be flipped for the current coral station
  private static boolean shouldFlipDriverOverride(SwerveSubsystem drive) {
    return findClosestCoralStation(drive)
            .equals(GlobalConstants.FieldMap.Coordinates.RIGHT_CORAL_STATION.getPose())
        ? false
        : true;
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(SwerveSubsystem drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
