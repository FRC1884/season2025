package frc.robot.subsystems;

import static frc.robot.Config.Subsystems.*;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;
import static frc.robot.subsystems.Superstructure.SuperStates.LEVEL_FOUR;
import static frc.robot.subsystems.Superstructure.SuperStates.LF_OUTTAKE;
import static frc.robot.subsystems.Superstructure.SuperStates.LF_FLICK;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.generic.arm.Arms;
import frc.robot.generic.elevators.Elevators;
import frc.robot.generic.rollers.Rollers;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algae.AlgaePivotSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberGoal;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeGoal;
import frc.robot.subsystems.coral.CoralPivotSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorGoal;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Superstructure extends SubsystemBase {
  /**
   * For use during autonomous and the Superstructure's periodic. The action (non-idle) entries end
   * in "ing" intentionally – if the robot is not in the state of actively transitioning between
   * states, it's idling.
   */
  private Supplier<Pose2d> drivePoseSupplier;

  private final Arms arms = new Arms();
  private final Elevators elevators = new Elevators();
  private final Rollers rollers = new Rollers();
  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  public Superstructure(Supplier<Pose2d> drivePoseSupplier) {
    this.drivePoseSupplier = drivePoseSupplier;
    if (LEDS_ENABLED)
      leds.setDefaultCommand(
          leds.ledCommand(
              DriverStation::isEnabled,
              DriverStation::isFMSAttached,
              () -> (DriverStation.getMatchTime() <= 30),
              () -> coralMode,
              () -> false,
              ALGAE_INTAKE_ENABLED ? rollers.getAlgaeIntake().hasAlgae() : () -> false,
              CORAL_INTAKE_ENABLED ? rollers.getCoralIntake().hasCoral() : () -> false));
  }

  public static enum SuperStates {
    IDLING,
    TESTING,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE,
    LEVEL_FOUR,
    OUTTAKE,
    INTAKE,
    STOP_INTAKE,
    SOURCE,
    LF_OUTTAKE,
    LF_FLICK,
    CLIMBER_DOWN,
    CLIMBER_UP,
    PROCESSOR
  }

  /**
   * For use during teleop to modify the current SuperState. Each one requests a state in {@link
   * SuperStates}. REQ_NONE is the absence of
   */
  public static enum StateRequests {
    REQ_NONE,
    REQ_IDLE,
    REQ_INTAKE,
    REQ_SHOOT
  }

  private boolean coralMode = true;

  public Command switchMode() {
    return Commands.runOnce(() -> coralMode = !coralMode);
  }

  private SuperStates currentState = IDLING;

  public Command setSuperStateCmd(SuperStates stateRequest) {
    return Commands.runOnce(() -> currentState = stateRequest);
  }

  /**
   * Periodically updates the goals of different subsystems based on the current superstructure
   * state. The state determines the actions of various subsystems such as coral and algae intake,
   * elevator, climber, and pivot. Each state sets specific goals for the subsystems, such as
   * idling, testing, moving to a certain level, or performing intake and outtake operations.
   */
  @Override
  public void periodic() {
    switch (currentState) {
      case IDLING -> {
        if (CORAL_INTAKE_ENABLED)
          rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.IDLING);
        if (ALGAE_INTAKE_ENABLED)
          rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.IDLING);
        if (ALGAE_PIVOT_ENABLED) arms.getAlgaePivot().setGoal(AlgaePivotSubsystem.PivotGoal.IDLING);
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.IDLING);
        if (CLIMBER_ENABLED) elevators.getClimber().setGoal(ClimberGoal.IDLING);
        if (CORAL_PIVOT_ENABLED) arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.IDLING);
        if (ALGAE_PIVOT_ENABLED) arms.getAlgaePivot().setGoal(AlgaePivotSubsystem.PivotGoal.IDLING);
      }
      case TESTING -> {
        if (CORAL_INTAKE_ENABLED)
          rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.FORWARD);
        if (ALGAE_INTAKE_ENABLED)
          rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.FORWARD);
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.TESTING);
        if (CORAL_PIVOT_ENABLED)
          arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.TESTING);
        if (ALGAE_PIVOT_ENABLED)
          arms.getAlgaePivot().setGoal(AlgaePivotSubsystem.PivotGoal.TESTING);
      }
      case LEVEL_ONE -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_ONE);
        if (CORAL_PIVOT_ENABLED)
          arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.LEVEL_ONE);
      }
      case LEVEL_TWO -> {
        if(coralMode) {
          if (ELEVATOR_ENABLED)
            elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_TWO);
          if (CORAL_PIVOT_ENABLED)
            arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.LEVEL_TWO);
        }
        else {

          if (CORAL_PIVOT_ENABLED)
            arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.DEALGAEFY_L3);
          if (CORAL_INTAKE_ENABLED) rollers.getCoralIntake().setGoal(CoralIntakeGoal.FORWARD);
          if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorGoal.DEALGAEFY_L3);
        }
      }
      case LEVEL_THREE -> {
        if(coralMode) {
          if (ELEVATOR_ENABLED)
            elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_THREE);
          if (CORAL_PIVOT_ENABLED)
            arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.LEVEL_THREE);
        }
        else {
          if (CORAL_PIVOT_ENABLED)
            arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.DEALGAEFY_L3);
          if (CORAL_INTAKE_ENABLED) rollers.getCoralIntake().setGoal(CoralIntakeGoal.FORWARD);
          if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorGoal.DEALGAEFY_L3);
        }
      }
      case LEVEL_FOUR -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_FOUR);
        if (CORAL_PIVOT_ENABLED)
          arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.LEVEL_FOUR);
      }
      case INTAKE -> {
        if (coralMode) {
          if (CORAL_INTAKE_ENABLED)
            rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.FORWARD);
        } else {
          if (ALGAE_INTAKE_ENABLED)
            rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.FORWARD);
          if (ALGAE_PIVOT_ENABLED)
            arms.getAlgaePivot().setGoal(AlgaePivotSubsystem.PivotGoal.INTAKE);
        }
      }
      case OUTTAKE -> {
        if (coralMode) {
          if (CORAL_INTAKE_ENABLED)
            rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.REVERSE);
        } else {
          if (ALGAE_INTAKE_ENABLED)
            rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.REVERSE);
          if (ALGAE_PIVOT_ENABLED)
            arms.getAlgaePivot().setGoal(AlgaePivotSubsystem.PivotGoal.PROCESSOR);
        }
      }
      case SOURCE -> {
        if (ELEVATOR_ENABLED)
          elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.SOURCE);
        if (CORAL_PIVOT_ENABLED) arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.SOURCE);
        if (CORAL_INTAKE_ENABLED)
          rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.FORWARD);
      }
      case STOP_INTAKE -> {
        if (coralMode) {
          if (CORAL_INTAKE_ENABLED)
            rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.IDLING);
        } else {
          if (ALGAE_INTAKE_ENABLED)
            rollers.getAlgaeIntake().setGoal(AlgaeIntakeSubsystem.AlgaeIntakeGoal.IDLING);
        }
      }
      case LF_OUTTAKE -> {
        if (CORAL_INTAKE_ENABLED)
          rollers.getCoralIntake().setGoal(CoralIntakeSubsystem.CoralIntakeGoal.LFOUTAKE);
      }
      case LF_FLICK -> {
        if (CORAL_PIVOT_ENABLED)
          arms.getCoralPivot().setGoal(CoralPivotSubsystem.PivotGoal.LEVEL_FOUR_FLICK);
      }
      case CLIMBER_DOWN -> {
        if (CLIMBER_ENABLED) elevators.getClimber().setGoal(ClimberGoal.FORWARD);
      }
      case CLIMBER_UP -> {
        if (CLIMBER_ENABLED) elevators.getClimber().setGoal(ClimberGoal.REVERSE);
      }
    }
  }

  public Trigger coralMode() {
    return new Trigger(() -> coralMode);
  }

  public Trigger hasGamePiece() {
    if (CORAL_INTAKE_ENABLED) {
      if (coralMode) return new Trigger(rollers.getCoralIntake().hasCoral());
    } else if (ALGAE_INTAKE_ENABLED) {
      return new Trigger(rollers.getAlgaeIntake().hasAlgae());
    } else return new Trigger(() -> false);
    return new Trigger(() -> false);
  }

  public Command lFScore() {
    return Commands.sequence(
        setSuperStateCmd(LEVEL_FOUR),
        Commands.waitSeconds(1.1),
        setSuperStateCmd(LF_OUTTAKE),
        Commands.waitSeconds(0.38),
        setSuperStateCmd(LF_FLICK));
  }

  public void registerSuperstructureCharacterization(
      Supplier<LoggedDashboardChooser<Command>> autoChooser) {}
}
