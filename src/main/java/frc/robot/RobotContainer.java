// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import static frc.robot.Config.Controllers.getDriverController;
import static frc.robot.Config.Controllers.getOperatorController;
import static frc.robot.Config.Subsystems.ALGAE_PIVOT_ENABLED;
import static frc.robot.Config.Subsystems.DRIVETRAIN_ENABLED;
import static frc.robot.Config.Subsystems.VISION_ENABLED;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.GlobalConstants.TUNING_MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.CLIMBER_DOWN;
import static frc.robot.subsystems.Superstructure.SuperStates.CLIMBER_UP;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;
import static frc.robot.subsystems.Superstructure.SuperStates.INTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_FOUR;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_ONE;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_THREE;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_TWO;
import static frc.robot.subsystems.Superstructure.SuperStates.OUTTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.STOP_INTAKE;
import static frc.robot.subsystems.swerve.SwerveConstants.BACK_LEFT;
import static frc.robot.subsystems.swerve.SwerveConstants.BACK_RIGHT;
import static frc.robot.subsystems.swerve.SwerveConstants.FRONT_LEFT;
import static frc.robot.subsystems.swerve.SwerveConstants.FRONT_RIGHT;
import static frc.robot.subsystems.swerve.SwerveConstants.GYRO_TYPE;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.BACK_CAM_CONSTANTS;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.BACK_CAM_ENABLED;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.LEFT_CAM_CONSTANTS;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.LEFT_CAM_ENABLED;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.RIGHT_CAM_CONSTANTS;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.RIGHT_CAM_ENABLED;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.OI.DriverMap;
import frc.robot.OI.OperatorMap;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperStates;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drive;
  private SwerveDriveSimulation driveSimulation;

  // Controller
  private final DriverMap driver = getDriverController();

  private final OperatorMap operator = getOperatorController();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Superstructure superstructure = new Superstructure(null);
  private final Vision vision;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (DRIVETRAIN_ENABLED) {
      drive =
          switch (MODE) {
            case REAL:
              // Real robot, instantiate hardware IO implementations
              yield new SwerveSubsystem(
                  switch (GYRO_TYPE) {
                    case PIGEON -> new GyroIOPigeon2();
                    case NAVX -> new GyroIONavX();
                    case ADIS -> new GyroIO() {};
                  },
                  new ModuleIOSpark(FRONT_LEFT),
                  new ModuleIOSpark(FRONT_RIGHT),
                  new ModuleIOSpark(BACK_LEFT),
                  new ModuleIOSpark(BACK_RIGHT));

            case SIM:
              // Create a maple-sim swerve drive simulation instance
              this.driveSimulation =
                  new SwerveDriveSimulation(
                      SwerveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
              // Add the simulated drivetrain to the simulation field
              SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

              // Sim robot, instantiate physics sim IO implementations
              yield new SwerveSubsystem(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOSim(driveSimulation.getModules()[0]),
                  new ModuleIOSim(driveSimulation.getModules()[1]),
                  new ModuleIOSim(driveSimulation.getModules()[2]),
                  new ModuleIOSim(driveSimulation.getModules()[3]));

            default:
              // Replayed robot, disable IO implementations
              yield new SwerveSubsystem(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
          };

      AutoCommands.registerAutoCommands(superstructure, drive);

      // Set up auto routines
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

      // Set up SysId routines
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    } else {
      drive = null;
      autoChooser = null;
    }

    if (VISION_ENABLED) {
      vision =
          switch (MODE) {
            case REAL -> new Vision(
                drive,
                LEFT_CAM_ENABLED
                    ? new AprilTagVisionIOPhotonVision(LEFT_CAM_CONSTANTS)
                    : new VisionIO() {},
                RIGHT_CAM_ENABLED
                    ? new AprilTagVisionIOPhotonVision(RIGHT_CAM_CONSTANTS)
                    : new VisionIO() {},
                BACK_CAM_ENABLED
                    ? new AprilTagVisionIOPhotonVision(BACK_CAM_CONSTANTS)
                    : new VisionIO() {});
            case SIM -> new Vision(
                drive,
                LEFT_CAM_ENABLED
                    ? new VisionIOPhotonVisionSim(
                        LEFT_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                    : new VisionIO() {},
                RIGHT_CAM_ENABLED
                    ? new VisionIOPhotonVisionSim(
                        RIGHT_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                    : new VisionIO() {},
                BACK_CAM_ENABLED
                    ? new VisionIOPhotonVisionSim(
                        BACK_CAM_CONSTANTS, driveSimulation::getSimulatedDriveTrainPose)
                    : new VisionIO() {});
            default -> new Vision(drive, new VisionIO() {}, new VisionIO() {});
          };
    } else vision = null;

    // Configure the button bindings
    configureDriverButtonBindings();
    configureOperatorButtonBindings();

    // Register the auto commands
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverButtonBindings() {
    if (DRIVETRAIN_ENABLED) {
      // Default command, normal field-relative drive
      drive.setDefaultCommand(
          Commands.run(
              () ->
                  DriveCommands.joystickDrive(
                      drive, driver.getYAxis(), driver.getXAxis(), driver.getRotAxis()),
              drive));

      // Lock to 0° when A button is held
      // driver
      //     .alignToSpeaker()
      //     .whileTrue(
      //         DriveCommands.joystickDriveAtAngle(
      //             drive, driver.getYAxis(), driver.getXAxis(), () -> new Rotation2d()));

      // Switch to X pattern when X button is pressed
      driver.stopWithX().onTrue(Commands.runOnce(drive::stopWithX, drive));

      // right align to reef face when right bumper is pressed and the robot is in coral mode
      driver.rightAlign().whileTrue(DriveCommands.rightAlignToReefCommandTeleop(drive));

      // left align to reef face when right numper is pressed and the robot is in coral mode
      driver.leftAlign().whileTrue(DriveCommands.leftAlignToReefCommandTeleop(drive));

      // align to coral station with position customization when right trigger is pressed
      driver.coralStation().whileTrue(DriveCommands.alignToNearestCoralStationCommandAuto(drive));

      PathConstraints constraints = new PathConstraints(0.5, 1, 0.5, 0.5);

      //   driver
      //       .leftAlign()
      //       .and(superstructure.coralMode().negate())
      //       .whileTrue(
      //           AutoBuilder.pathfindToPose(
      //               GlobalConstants.FieldMap.Coordinates.PROCESSOR.getPose(), constraints));

      driver
          .slowMode()
          .whileTrue(
              Commands.run(
                  () ->
                      DriveCommands.joystickDrive(
                          drive,
                          () -> driver.getYAxis().getAsDouble() / 3.0,
                          () -> driver.getXAxis().getAsDouble() / 3.0,
                          () -> driver.getRotAxis().getAsDouble() / 3.0),
                  drive));

      // Reset gyro to 0° when B button is pressed
      driver
          .resetOdometry()
          .onTrue(
              Commands.runOnce(
                      () ->
                          drive.resetOdometry(
                              new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                      drive)
                  .ignoringDisable(true));
    }
  }

  private void configureOperatorButtonBindings() {
    if (ALGAE_PIVOT_ENABLED) {
      if (TUNING_MODE) {
        operator.levelOne().onTrue(superstructure.setSuperStateCmd(SuperStates.TESTING));
      } else {
        operator.levelOne().onTrue(superstructure.setSuperStateCmd(LEVEL_ONE));
      }

      operator.levelTwo().onTrue(superstructure.setSuperStateCmd(LEVEL_TWO));

      operator.levelThree().onTrue(superstructure.setSuperStateCmd(LEVEL_THREE));

      operator.levelFour().onTrue(superstructure.setSuperStateCmd(LEVEL_FOUR));

      operator
          .climberUp()
          .onTrue(superstructure.setSuperStateCmd(CLIMBER_UP))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .climberDown()
          .onTrue(superstructure.setSuperStateCmd(CLIMBER_DOWN))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .intake()
          .onTrue(
              Commands.sequence(
                  superstructure.setSuperStateCmd(INTAKE),
                  Commands.waitUntil(
                      new Trigger(
                          () ->
                              (superstructure.hasGamePiece().getAsBoolean()
                                  || !operator.intake().getAsBoolean()))),
                  superstructure.setSuperStateCmd(STOP_INTAKE)));

      operator
          .outtake()
          .onTrue(
              Commands.sequence(
                  superstructure.setSuperStateCmd(OUTTAKE),
                  Commands.waitUntil(
                      new Trigger(
                          () ->
                              (superstructure.hasGamePiece().getAsBoolean()
                                  || !operator.outtake().getAsBoolean()))),
                  superstructure.setSuperStateCmd(STOP_INTAKE)));

      operator.modeSwitch().onTrue(superstructure.setSuperStateCmd(IDLING));
    } else {
      if (TUNING_MODE) {
        operator
            .levelOne()
            .onTrue(superstructure.setSuperStateCmd(SuperStates.TESTING))
            .onFalse(superstructure.setSuperStateCmd(IDLING));
      } else {
        operator
            .levelOne()
            .onTrue(superstructure.setSuperStateCmd(LEVEL_ONE))
            .onFalse(superstructure.setSuperStateCmd(IDLING));
      }

      operator
          .levelTwo()
          .onTrue(superstructure.setSuperStateCmd(LEVEL_TWO))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .levelThree()
          .onTrue(superstructure.setSuperStateCmd(LEVEL_THREE))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .levelFour()
          .onTrue(
              superstructure.lFScore(
                  () -> operator.outtake().getAsBoolean(),
                  () -> !operator.levelFour().getAsBoolean()))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .climberUp()
          .onTrue(superstructure.setSuperStateCmd(CLIMBER_UP))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .climberDown()
          .onTrue(superstructure.setSuperStateCmd(CLIMBER_DOWN))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .intake()
          .onTrue(superstructure.setSuperStateCmd(INTAKE))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator
          .outtake()
          .onTrue(superstructure.setSuperStateCmd(OUTTAKE))
          .onFalse(superstructure.setSuperStateCmd(IDLING));

      operator.modeSwitch().onTrue(superstructure.switchMode());
      operator.resetElevator().onTrue(superstructure.resetElevator());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous, or null if the auto chooser is not initialized.
   */
  public Command getAutonomousCommand() {
    if (autoChooser == null) return null;
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (MODE != RobotMode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
