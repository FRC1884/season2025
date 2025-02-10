// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Arrays;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private record LEDOutputValue(LEDPattern pattern, int index) {
    LEDOutputValue(LEDPattern pattern) {
      this(pattern, -1);
    }

    static LEDOutputValue[] all(LEDPattern pattern) {
      return new LEDOutputValue[]{new LEDOutputValue(pattern)};
    }
  }


  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  private double rainbowStart = 0;
  private double dashStart = 0;

  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem(LEDIO io) {
    super();
    this.io = io;
  }

  @Override
  public void periodic() {
    io.periodic();
    Logger.processInputs("LED", inputs);
  }


  private void setPattern(LEDOutputValue[] values) {
    if (Arrays.stream(values).anyMatch(value -> value.index == -1)) {
      var opt = Arrays.stream(values).filter(value -> value.index == -1).findFirst();

      if (opt.isPresent()) {
        var value = opt.get();

        io.setAllPattern(value.pattern);
      }
    }

    for (LEDOutputValue value : values) {
      io.setPattern(value.index(), value.pattern());
    }
  }

  public Command ledCommand(BooleanSupplier isEnabled, BooleanSupplier hasFMS, BooleanSupplier isEndgame, BooleanSupplier isCoral, BooleanSupplier isAligned, BooleanSupplier hasAlgae, BooleanSupplier hasCoral) {
    return this.run(() -> {
      if (isEnabled.getAsBoolean()) {
        var patterns = getPattern(isEndgame.getAsBoolean(), isCoral.getAsBoolean(), isAligned.getAsBoolean(), hasAlgae.getAsBoolean(), hasCoral.getAsBoolean());
        setPattern(patterns);
      } else if (!hasFMS.getAsBoolean()) {
        setPattern(LEDOutputValue.all(LEDPattern.solid(Color.kBlue).breathe(Second.of(1))));
      }
      else {
        // get pose, get first pose of auto somehow
        var patterns = getFieldAlignPattern(new Pose2d(), new Pose2d());
        setPattern(patterns);
      }

    });
  }

  private enum ErrDir {
    POS, NEG, NONE;
  }

  private ErrDir getError(double error, double tolerance) {
    if (error > tolerance) {
      return ErrDir.POS;
    } else if (error < -tolerance) {
      return ErrDir.NEG;
    } else {
      return ErrDir.NONE;
    }
  }

  private LEDOutputValue[] getFieldAlignPattern(Pose2d realPose, Pose2d targetPose) {
    var errors = targetPose.relativeTo(realPose);

    var dist = Math.min(Math.abs(targetPose.getTranslation().getDistance(realPose.getTranslation())), FLASHING_MAX);
    var blinkSpeed = Second.of(1 / (5 * dist));

    var xError = getError(errors.getTranslation().getX(), TRANSLATION_TOLERANCE);
    var yError = getError(errors.getTranslation().getY(), TRANSLATION_TOLERANCE);
    var rotError = getError(errors.getRotation().getRadians(), ROTATION_TOLERANCE);

    if (xError == ErrDir.NONE && yError == ErrDir.NONE && rotError == ErrDir.NONE) {
      return LEDOutputValue.all(LEDPattern.solid(OK_COLOR));
    }

    if (rotError != ErrDir.NONE) {
      // check if CW or CCW
      if (rotError == ErrDir.POS) {
        return LEDOutputValue.all(LEDPattern.solid(MORE_COLOR));
      } else {
        return LEDOutputValue.all(LEDPattern.solid(LESS_COLOR));
      }
    }

    var frontLeft = (xError == ErrDir.POS || yError == ErrDir.POS) ? LEDPattern.solid(MORE_COLOR) : LEDPattern.solid(LESS_COLOR);
    var frontRight = (xError == ErrDir.POS || yError == ErrDir.NEG) ? LEDPattern.solid(MORE_COLOR) : LEDPattern.solid(LESS_COLOR);
    var backLeft = (xError == ErrDir.NEG || yError == ErrDir.POS) ? LEDPattern.solid(MORE_COLOR) : LEDPattern.solid(LESS_COLOR);
    var backRight = (xError == ErrDir.NEG || yError == ErrDir.NEG) ? LEDPattern.solid(MORE_COLOR) : LEDPattern.solid(LESS_COLOR);

    return new LEDOutputValue[] {
      new LEDOutputValue(frontLeft.breathe(blinkSpeed), FRONT_LEFT),
      new LEDOutputValue(frontRight.breathe(blinkSpeed), FRONT_RIGHT),
      new LEDOutputValue(backLeft.breathe(blinkSpeed), BACK_LEFT),
      new LEDOutputValue(backRight.breathe(blinkSpeed), BACK_RIGHT)
    };
  }

  private LEDOutputValue[] getPattern(Boolean isEndgame, Boolean isCoral, Boolean isAligned, Boolean hasAlgae, Boolean hasCoral) {
    if (isEndgame) {
      return endgameColor();
    } else {
      return teleopColor(isCoral, isAligned, hasAlgae, hasCoral);
    }
  }

  private LEDOutputValue[] endgameColor() {
    return LEDOutputValue.all(LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
  }

  private LEDOutputValue[] teleopColor(Boolean isCoral, Boolean isAligned, Boolean hasAlgae, Boolean hasCoral) {
    if (isCoral) {
      return gamePieceColor(CORAL_COLOR, isAligned, hasCoral);
    } else {
      return gamePieceColor(ALGAE_COLOR, isAligned, hasAlgae);
    }
  }

  private LEDOutputValue[] gamePieceColor(Color color, Boolean isAligned, Boolean hasPiece) {
    if (isAligned) {
      return LEDOutputValue.all(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color, Color.kBlack).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
    } else if (hasPiece) {
      return LEDOutputValue.all(LEDPattern.solid(color));
    } else {
      return LEDOutputValue.all(LEDPattern.solid(color).breathe(Second.of(1)));
    }
  }


}


