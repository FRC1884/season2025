package frc.robot.subsystems;

import static frc.robot.Config.Subsystems.*;
import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.generic.arm.Arms;
import frc.robot.generic.elevators.Elevators;
import frc.robot.generic.rollers.Rollers;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.intakeCoral.IntakeCoralSubsystem;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

import java.sql.Driver;
import java.util.function.Supplier;

import frc.robot.util.AllianceFlipUtil;
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
      leds.setDefaultCommand(leds.ledCommand(
              DriverStation::isEnabled,
              DriverStation::isFMSAttached,
              ()->  (DriverStation.getMatchTime()<=30),
              ()->  true,
              ()->  false,
              INTAKE_ALGAE_ENABLED  ? rollers.getIntakeAlgae().hasAlgae() : ()->  false,
              INTAKE_CORAL_ENABLED  ? rollers.getIntakeCoral().hasCoral() : ()->  false
              ));
  }


  public static enum SuperStates {
    IDLING,
    TESTING,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE,
    LEVEL_FOUR,
    INTAKE_ALGAE,
    OUTAKE_ALGAE,
    INTAKE_CORAL,
    OUTAKE_CORAL
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

  private SuperStates currentState = IDLING;

  public Command setSuperStateCmd(SuperStates stateRequest) {
    return Commands.run(() -> currentState = stateRequest);
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLING -> {
        if (INTAKE_CORAL_ENABLED) rollers.getIntakeCoral().setGoal(IntakeCoralSubsystem.IntakeCoralGoal.IDLING);
        if (INTAKE_ALGAE_ENABLED) rollers.getIntakeAlgae().setGoal(IntakeAlgaeSubsystem.IntakeAlgaeGoal.IDLING);
        if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.IDLING);
        if (CLIMBER_ENABLED) elevators.getClimber().setGoal(ClimberSubsystem.ClimberGoal.IDLING);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.IDLING);
      }
      case TESTING -> {
        if (INTAKE_CORAL_ENABLED) rollers.getIntakeCoral().setGoal(IntakeCoralSubsystem.IntakeCoralGoal.FORWARD);
        if (INTAKE_ALGAE_ENABLED) rollers.getIntakeAlgae().setGoal(IntakeAlgaeSubsystem.IntakeAlgaeGoal.FORWARD);
        if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.TESTING);
        if (CLIMBER_ENABLED) elevators.getClimber().setGoal(ClimberSubsystem.ClimberGoal.TESTING);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.TESTING);
      }
      case LEVEL_ONE -> {
        if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_ONE);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_ONE);
      }
      case LEVEL_TWO -> {
        if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_TWO);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_TWO);
      }
      case LEVEL_THREE -> {
        if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_THREE);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_THREE);
      }
      case LEVEL_FOUR -> {
        if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.LEVEL_FOUR);
        if (PIVOT_ENABLED) arms.getPivot().setGoal(PivotSubsystem.PivotGoal.LEVEL_FOUR);
      }
      case INTAKE_ALGAE -> {
        if (INTAKE_ALGAE_ENABLED) rollers.getIntakeAlgae().setGoal(IntakeAlgaeSubsystem.IntakeAlgaeGoal.FORWARD);
      }
      case OUTAKE_ALGAE -> {
        if (INTAKE_ALGAE_ENABLED) rollers.getIntakeAlgae().setGoal(IntakeAlgaeSubsystem.IntakeAlgaeGoal.REVERSE);
      }
      case INTAKE_CORAL -> {
        if (INTAKE_CORAL_ENABLED) rollers.getIntakeCoral().setGoal(IntakeCoralSubsystem.IntakeCoralGoal.FORWARD);
        if (ELEVATOR_ENABLED) elevators.getElevator().setGoal(ElevatorSubsystem.ElevatorGoal.TESTING);
      }
      case OUTAKE_CORAL -> {
        if (INTAKE_CORAL_ENABLED) rollers.getIntakeCoral().setGoal(IntakeCoralSubsystem.IntakeCoralGoal.REVERSE);
      }
    }
    }

  public void registerSuperstructureCharacterization(
      Supplier<LoggedDashboardChooser<Command>> autoChooser) {}
}
