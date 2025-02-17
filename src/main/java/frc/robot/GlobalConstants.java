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

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.loadField;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.RotationalAllianceFlipUtil;
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
  public static final double ODOMETRY_FREQUENCY = 100.0;

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
      Alert alert =
          new Alert(
              "SIM robot loaded in REAL mode, gains likely breaking!", Alert.AlertType.kWarning);
      alert.set(true);
    }
  }

  // Blue origin, so we use blue side coords
  public static final class FieldMap {
    // TODO Change coordinates based on robot offset
    @Getter
    public static enum Coordinates {
      REEF_1(new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(0))),
      REEF_2(new Pose2d(4.073905999999999, 3.3063179999999996, Rotation2d.fromDegrees(60))),
      REEF_3(new Pose2d(4.904739999999999, 3.3063179999999996, Rotation2d.fromDegrees(120))),
      REEF_4(new Pose2d(5.321046, 4.0259, Rotation2d.fromDegrees(180))),
      REEF_5(new Pose2d(4.904739999999999, 4.745482, Rotation2d.fromDegrees(240))),
      REEF_6(new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromDegrees(300))),
      LEFT_CORAL_STATION(
          new Pose2d(0.851154, 7.3964799999999995, Rotation2d.fromDegrees(-54.011392 + 180))),
      RIGHT_CORAL_STATION(new Pose2d(0.851154, 0.65532, Rotation2d.fromDegrees(54.011392 + 180))),
      PROCESSOR(new Pose2d(5.9875419999999995, -0.0038099999999999996, Rotation2d.fromDegrees(90)));

      private final Pose2d pose;

      Coordinates(Pose2d pose) {
        this.pose = RotationalAllianceFlipUtil.apply(pose);
      }
    }

    public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26 + (5.0 / 12));
    public static final double FIELD_LENGTH_METERS = Units.feetToMeters(57 + (6.875 / 12));

    public static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    static {
      try {
        // Path reads from the working directory, and splices at `/src/main/deploy/`
        APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(Path.of("tagfields/home_testing_1.json"));
      } catch (IOException e) {
        System.err.println("Custom tag map not found, using default layout!");
        APRIL_TAG_FIELD_LAYOUT = loadField(AprilTagFields.kDefaultField);
      }
    }
  }

  public static final class AlignOffsets {
    public static final double BUMPER_TO_CENTER_OFFSET =
        Units.inchesToMeters(ROBOT == RobotType.DEVBOT ? -(26.0 / 2 + 3) : -(28.0 / 2 + 3));
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
