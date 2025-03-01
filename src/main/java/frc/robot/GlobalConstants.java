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

import static edu.wpi.first.apriltag.AprilTagFieldLayout.loadField;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.RotationalAllianceFlipUtil;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class GlobalConstants {
  public static final RobotMode MODE = RobotMode.REAL;
  public static final RobotType ROBOT = RobotType.DEVBOT;
  public static final double ODOMETRY_FREQUENCY = 250.0;

  public static boolean TUNING_MODE = true;

  public static enum RobotMode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY,
  }

  public static enum RobotType {
    COMPBOT,
    DEVBOT,
    SIMBOT
  }

  /**
   * Checks whether the correct robot is selected when deploying (the main method is only ever
   * called on deploy).
   */
  public static void main(String... args) {
    if (ROBOT == RobotType.SIMBOT) {
      new Alert("SIM robot loaded in REAL mode, gains likely breaking!", AlertType.kWarning)
          .set(true);
    }
  }

  // Blue origin, so we use blue side coords and tags
  public static final class FieldMap {
    public static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;
    public static final boolean WELDED_FIELD = false;

    static {
      try {
        // Path reads from the working directory, and splices at `/src/main/deploy/`
        APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(Path.of("tagfields/home_testing_1.json"));
      } catch (IOException e) {
        new Alert("Custom tag map not found, using default layout!", AlertType.kWarning).set(true);
        APRIL_TAG_FIELD_LAYOUT =
            loadField(
                WELDED_FIELD
                    ? AprilTagFields.k2025ReefscapeWelded
                    : AprilTagFields.k2025ReefscapeAndyMark);
      }
      // APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(Path.of("tagfields/home_testing_1.json"));
    }

    // TODO set all alignment offsets from tags
    @Getter
    public static enum Coordinates {
      REEF_1(APRIL_TAG_FIELD_LAYOUT.getTagPose(18).get().toPose2d()),
      REEF_2(APRIL_TAG_FIELD_LAYOUT.getTagPose(17).get().toPose2d()),
      REEF_3(APRIL_TAG_FIELD_LAYOUT.getTagPose(22).get().toPose2d()),
      REEF_4(APRIL_TAG_FIELD_LAYOUT.getTagPose(21).get().toPose2d()),
      REEF_5(APRIL_TAG_FIELD_LAYOUT.getTagPose(20).get().toPose2d()),
      REEF_6(APRIL_TAG_FIELD_LAYOUT.getTagPose(19).get().toPose2d()),
      LEFT_CORAL_STATION(APRIL_TAG_FIELD_LAYOUT.getTagPose(13).get().toPose2d()),
      RIGHT_CORAL_STATION(APRIL_TAG_FIELD_LAYOUT.getTagPose(12).get().toPose2d()),
      PROCESSOR(APRIL_TAG_FIELD_LAYOUT.getTagPose(16).get().toPose2d()),
      LEFT_BARGE(APRIL_TAG_FIELD_LAYOUT.getTagPose(14).get().toPose2d()),
      RIGHT_BARGE(APRIL_TAG_FIELD_LAYOUT.getTagPose(15).get().toPose2d());

      private final Pose2d pose;

      Coordinates(Pose2d pose) {
        this.pose =
            RotationalAllianceFlipUtil.apply(
                pose.rotateAround(pose.getTranslation(), Rotation2d.k180deg));
      }
    }

    public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26 + (5.0 / 12));
    public static final double FIELD_LENGTH_METERS = Units.feetToMeters(57 + (6.875 / 12));
  }

  public static final class AlignOffsets {
    public static final double BUMPER_TO_CENTER_OFFSET =
        Units.inchesToMeters(ROBOT == RobotType.DEVBOT ? -(26.0 / 2 + 3) : -(28.0 / 2 + 3.5));

    public static final double REEF_TO_BUMPER_OFFSET = -27.0 / 100;
    public static final double REEF_TO_BRANCH_OFFSET = Units.inchesToMeters(13.0 / 2);
  }

  /** PID + FF gains, with overloaded constructors for disabling each term. */
  public record Gains(double kP, double kD, double kS, double kV, double kA, double kG) {
    public Gains(double kP, double kD, double kS, double kV, double kA) {
      this(kP, kD, kS, kV, kA, 0.0);
    }

    public Gains(double kP, double kD, double kS, double kV) {
      this(kP, kD, kS, kV, 0.0);
    }

    public Gains(double kP, double kD) {
      this(kP, kD, 0.0, 0.0);
    }
  }
}
