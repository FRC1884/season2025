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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
  public static void joystickDrive(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

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
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  public static void robotRelativeChassisSpeedDrive(SwerveSubsystem drive, ChassisSpeeds speeds) {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(speeds);
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
    TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);
    TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);
    // TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);

    ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0.0, X_CONSTRAINTS);
    ProfiledPIDController yController = new ProfiledPIDController(1.5, 0, 0.0, Y_CONSTRAINTS);
    PIDController omegaPID = new PIDController(0.03, 0, 0.0);

    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    omegaPID.setTolerance(0.05);
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
                  DriveCommands.robotRelativeChassisSpeedDrive(
                      drive, new ChassisSpeeds(xSpeed * 1.2, ySpeed * 1.2, omegaSpeed * 1.2));
                },
                interrupted -> {
                  DriveCommands.robotRelativeChassisSpeedDrive(drive, new ChassisSpeeds());
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
    TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);
    TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 10);
    // TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);

    ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0.0, X_CONSTRAINTS);
    ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0.0, Y_CONSTRAINTS);
    PIDController omegaPID = new PIDController(0.03, 0, 0.0);

    xController.setTolerance(0.05);
    yController.setTolerance(0.03);
    omegaPID.setTolerance(0.2);
    omegaPID.enableContinuousInput(-180, 180);

    return new DeferredCommand(
        () ->
            new FunctionalCommand(
                () -> {
                  // Init
                },
                () -> {
                  double driverInputFactor = 1;
                  double ySpeed = yDriver.getAsDouble() * driverInputFactor;
                  double xSpeed = xController.calculate(0, targetOffset.get().getX());
                  double omegaSpeed =
                      omegaPID.calculate(0, targetOffset.get().getRotation().getDegrees());
                  DriveCommands.robotRelativeChassisSpeedDrive(
                      drive, new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
                },
                interrupted -> {
                  DriveCommands.robotRelativeChassisSpeedDrive(drive, new ChassisSpeeds());
                  omegaPID.close();
                  System.out.println("aligned now");
                },
                () -> {
                  return false;
                },
                drive),
        Set.of(drive));
  }

  // run the pathfind then follow command and then use PID to align on termination
  public static Command pathfindThenPIDCommand(SwerveSubsystem drive, Supplier<Pose2d> target) {
    Supplier<Transform2d> targetOffset = () -> target.get().minus(drive.getPose());

    PathConstraints constraints = new PathConstraints(0.5, 1, 0.5, 0.5);

    double endVelocity = 0.0;

    return Commands.sequence(
        AutoBuilder.pathfindToPose(target.get(), constraints, endVelocity),
        chasePoseRobotRelativeCommand(drive, targetOffset));
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

  /** align to processor */
  public static Command alignToProcessorCommand() {
    PathConstraints constraints = new PathConstraints(10, 5, 5, 5);
    Pose2d target = GlobalConstants.FieldMap.Coordinates.PROCESSOR.getPose();
    Transform2d offset =
        new Transform2d(
            new Translation2d(0, GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET)
                .rotateBy(target.getRotation()),
            new Rotation2d());
    target = target.plus(offset);
    Logger.recordOutput("Targets/Processor", target);
    return AutoBuilder.pathfindToPose(target, constraints);
  }

  /** code for reef alignment */
  public static Command leftAlignToReefCommandTeleop(SwerveSubsystem drive) {
    return alignToReefCommandTeleop(drive, () -> true, () -> 0);
  }

  public static Command rightAlignToReefCommandTeleop(SwerveSubsystem drive) {
    return alignToReefCommandTeleop(drive, () -> false, () -> 0);
  }

  /** helper methods for alignment */

  // align to target face
  public static Command alignToReefCommandAuto(
      SwerveSubsystem drive, BooleanSupplier leftInput, Supplier<Integer> targetReefFace) {
    return Commands.defer(
        () -> {
          // find the coordinates of the selected face
          Supplier<Pose2d> targetFace;
          targetFace =
              switch (targetReefFace.get()) {
                case 1 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_1.getPose();
                case 2 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_2.getPose();
                case 3 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_3.getPose();
                case 4 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_4.getPose();
                case 5 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_5.getPose();
                case 6 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_6.getPose();
                default -> findClosestReefFace(drive);
              };

          double xOffset =
              GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET
                  + GlobalConstants.AlignOffsets.REEF_TO_BUMPER_OFFSET;

          BooleanSupplier leftAlign = isFieldRelativeLeftAlign(targetFace, leftInput);
          double yOffset =
              GlobalConstants.AlignOffsets.REEF_TO_BRANCH_OFFSET
                  * (leftAlign.getAsBoolean() ? 1 : -1);
          Rotation2d rotation = targetFace.get().getRotation();
          Translation2d branchTransform = new Translation2d(xOffset, yOffset).rotateBy(rotation);
          Supplier<Pose2d> target =
              () ->
                  new Pose2d(
                      targetFace.get().getTranslation().plus(branchTransform),
                      targetFace.get().getRotation());

          Supplier<Transform2d> targetOffset = () -> target.get().minus(drive.getPose());

          boolean outsideApproach =
              leftAlign.getAsBoolean()
                  ? targetOffset.get().getMeasureY().magnitude() < 0
                  : targetOffset.get().getMeasureY().magnitude() > 0;

          Supplier<Transform2d> correctedTargetOffset =
              () ->
                  new Transform2d(
                      new Translation2d(
                          targetOffset.get().getMeasureX().magnitude(),
                          targetOffset.get().getMeasureY().magnitude()),
                      targetOffset.get().getRotation());

          return chasePoseRobotRelativeCommand(drive, correctedTargetOffset);
        },
        Set.of(drive));
  }

  // align to target face
  public static Command alignToReefCommandTeleop(
      SwerveSubsystem drive, BooleanSupplier leftInput, Supplier<Integer> targetReefFace) {
    return Commands.defer(
        () -> {
          // find the coordinates of the selected face
          Supplier<Pose2d> targetFace;
          targetFace =
              switch (targetReefFace.get()) {
                case 1 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_1.getPose();
                case 2 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_2.getPose();
                case 3 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_3.getPose();
                case 4 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_4.getPose();
                case 5 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_5.getPose();
                case 6 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_6.getPose();
                default -> findClosestReefFace(drive);
              };

          double xOffset =
              GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET
                  + GlobalConstants.AlignOffsets.REEF_TO_BUMPER_OFFSET;

          BooleanSupplier leftAlign = isFieldRelativeLeftAlign(targetFace, leftInput);
          double yOffset =
              GlobalConstants.AlignOffsets.REEF_TO_BRANCH_OFFSET
                  * (leftAlign.getAsBoolean() ? 1 : -1);
          Rotation2d rotation = targetFace.get().getRotation();
          Translation2d branchTransform = new Translation2d(xOffset, yOffset).rotateBy(rotation);
          Supplier<Pose2d> target =
              () ->
                  new Pose2d(
                      targetFace.get().getTranslation().plus(branchTransform),
                      targetFace.get().getRotation());

          Supplier<Transform2d> targetOffset = () -> target.get().minus(drive.getPose());

          boolean outsideApproach =
              leftAlign.getAsBoolean()
                  ? targetOffset.get().getMeasureY().magnitude() < 0
                  : targetOffset.get().getMeasureY().magnitude() > 0;

          // this number is the offset for approaching from the inside
          double directionalIncrease = 0.15;
          Supplier<Transform2d> correctedTargetOffset;
          correctedTargetOffset =
              () ->
                  new Transform2d(
                      new Translation2d(
                          targetOffset.get().getMeasureX().magnitude(),
                          targetOffset.get().getMeasureY().magnitude() + directionalIncrease),
                      targetOffset.get().getRotation());

          return chasePoseRobotRelativeCommand(drive, correctedTargetOffset);
        },
        Set.of(drive));
  }

  private static BooleanSupplier isFieldRelativeLeftAlign(
      Supplier<Pose2d> targetReefFace, BooleanSupplier leftInput) {
    boolean facingDriver =
        RotationalAllianceFlipUtil.apply(targetReefFace.get()).getRotation().getRadians()
                >= Math.PI / 2
            && RotationalAllianceFlipUtil.apply(targetReefFace.get()).getRotation().getRadians()
                <= 3 * Math.PI / 2;

    Logger.recordOutput("Targets/Facing Driver", facingDriver);

    return () -> facingDriver ? !leftInput.getAsBoolean() : leftInput.getAsBoolean();
  }

  // returns the nearest face of the reef
  public static Supplier<Pose2d> findClosestReefFace(SwerveSubsystem drive) {
    double reef1 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF_1.getPose().getTranslation());
    double reef2 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF_2.getPose().getTranslation());
    double reef3 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF_3.getPose().getTranslation());
    double reef4 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF_4.getPose().getTranslation());
    double reef5 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF_5.getPose().getTranslation());
    double reef6 =
        drive
            .getPose()
            .getTranslation()
            .getDistance(GlobalConstants.FieldMap.Coordinates.REEF_6.getPose().getTranslation());

    double closestFace =
        Math.min(Math.min(Math.min(reef1, reef2), Math.min(reef3, reef4)), Math.min(reef5, reef6));
    if (reef1 == closestFace) return () -> GlobalConstants.FieldMap.Coordinates.REEF_1.getPose();
    if (reef2 == closestFace) return () -> GlobalConstants.FieldMap.Coordinates.REEF_2.getPose();
    if (reef3 == closestFace) return () -> GlobalConstants.FieldMap.Coordinates.REEF_3.getPose();
    if (reef4 == closestFace) return () -> GlobalConstants.FieldMap.Coordinates.REEF_4.getPose();
    if (reef5 == closestFace) return () -> GlobalConstants.FieldMap.Coordinates.REEF_5.getPose();
    return () -> GlobalConstants.FieldMap.Coordinates.REEF_6.getPose();
  }
  /**
   * Command to align to the nearest coral station
   *
   * @param drive
   * @return
   */
  public static Command alignToNearestCoralStationCommand(
      SwerveSubsystem drive, DoubleSupplier yDriver) {
    return Commands.defer(
        () -> {
          DoubleSupplier driver =
              () -> yDriver.getAsDouble() * (shouldFlipDriverOverride(drive) ? -1 : 1);
          Supplier<Pose2d> target = () -> findClosestCoralStation(drive);
          Supplier<Transform2d> bumperOffset =
              () ->
                  new Transform2d(
                      new Translation2d(0, GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET)
                          .rotateBy(target.get().getRotation()),
                      new Rotation2d());
          Supplier<Transform2d> robotRelativeOffset =
              () -> target.get().minus(drive.getPose()).plus(bumperOffset.get());
          return chasePoseRobotRelativeCommandYOverride(drive, robotRelativeOffset, driver);
        },
        Set.of(drive));
  }

  public static Command alignToNearestCoralStationCommandAuto(SwerveSubsystem drive) {
    return Commands.defer(
        () -> {
          Supplier<Pose2d> target = () -> findClosestCoralStation(drive);
          Supplier<Transform2d> bumperOffset =
              () ->
                  new Transform2d(
                      new Translation2d(0, GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET)
                          .rotateBy(target.get().getRotation()),
                      new Rotation2d());
          Supplier<Transform2d> sideToSideOffset =
              () ->
                  new Transform2d(
                      new Translation2d(GlobalConstants.AlignOffsets.SIDE_TO_SIDE_OFFSET_AUTO, 0)
                          .rotateBy(target.get().getRotation()),
                      new Rotation2d());
          Supplier<Transform2d> robotRelativeOffset =
              () ->
                  target
                      .get()
                      .minus(drive.getPose())
                      .plus(bumperOffset.get())
                      .plus(sideToSideOffset.get());
          return chasePoseRobotRelativeCommand(drive, robotRelativeOffset);
        },
        Set.of(drive));
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
