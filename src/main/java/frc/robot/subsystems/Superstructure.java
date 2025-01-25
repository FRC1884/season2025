package frc.robot.subsystems;

import static frc.robot.GlobalConstants.MODE;
import static frc.robot.subsystems.Superstructure.SuperStates.IDLING;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.leds.LEDIOPWM;
import frc.robot.subsystems.leds.LEDIOSim;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Superstructure extends SubsystemBase {
  /**
   * For use during autonomous and the Superstructure's periodic. The action (non-idle) entries end
   * in "ing" intentionally – if the robot is not in the state of actively transitioning between
   * states, it's idling.
   */
  private Supplier<Pose2d> drivePoseSupplier;

  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  private final PivotSubsystem pivot =
      Config.Subsystems.PIVOT_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new PivotSubsystem(new PivotIOReal())
              : new PivotSubsystem(new PivotIOSim()))
          : null;
  private final ClimberSubsystem climber =
      Config.Subsystems.CLIMBER_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new ClimberSubsystem(new ClimberIOReal())
              : new ClimberSubsystem(new ClimberIOSim()))
          : null;

  public Superstructure(Supplier<Pose2d> drivePoseSupplier) {
    this.drivePoseSupplier = drivePoseSupplier;
  }

  public static enum SuperStates {
    IDLING,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE,
    LEVEL_FOUR,
    SOURCE,
    SHALLOW_CLIMB,
    DEEP_CLIMB
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
        if (leds != null)
          leds.setRunAlongCmd(
              () -> AllianceFlipUtil.shouldFlip() ? Color.kRed : Color.kBlue,
              () -> Color.kBlack,
              5,
              1);
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.IDLING);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.IDLING);
      }
      case LEVEL_ONE -> {
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.LEVEL_ONE);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.IDLING);
      }
      case LEVEL_TWO -> {
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.LEVEL_TWO);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.IDLING);
      }
      case LEVEL_THREE -> {
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.LEVEL_THREE);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.IDLING);
      }
      case LEVEL_FOUR -> {
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.LEVEL_FOUR);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.IDLING);
      }
      case SOURCE -> {
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.SOURCE);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.IDLING);
      }
      case DEEP_CLIMB -> {
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.IDLING);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.DEEP_CLIMB);
      }
      case SHALLOW_CLIMB -> {
        if (pivot != null) pivot.setGoal(PivotSubsystem.PivotGoal.IDLING);
        if (climber != null) climber.setGoal(ClimberSubsystem.ClimberGoal.SHALLOW_CLIMB);
      }
    }
  }

  public Command setLEDBlinkingCmd(Color onColor, Color offColor, double frequency) {
    if (leds != null) return leds.setBlinkingCmd(onColor, offColor, frequency);
    else return null;
  }

  public void registerSuperstructureCharacterization(
      Supplier<LoggedDashboardChooser<Command>> autoChooser) {}
}
