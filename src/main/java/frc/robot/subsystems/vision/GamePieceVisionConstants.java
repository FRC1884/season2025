package frc.robot.subsystems.vision;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class GamePieceVisionConstants {

  static final double GAME_PIECE_HEIGHT = Units.inchesToMeters(5.0);

  // 2D stuff
  static final Transform2d ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D =
      new Transform2d(0, 0, Rotation2d.fromDegrees(180));

  // 3D stuff
  static final double DETECTOR_LIMELIGHT_PITCH = degreesToRadians(20);
  static final double DETECTOR_LIMELIGHT_ROLL = degreesToRadians(0.0);
  static final double DETECTOR_LIMELIGHT_HEIGHT = 0.9;

  static final boolean NOTE_CAM_ENABLED = true;

  static final Transform3d ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_3D =
      new Transform3d(
          ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.getX(),
          ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.getY(),
          DETECTOR_LIMELIGHT_HEIGHT,
          new Rotation3d(
              DETECTOR_LIMELIGHT_ROLL,
              DETECTOR_LIMELIGHT_PITCH,
              ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_2D.getRotation().getRadians()));

  public static final VisionIO.CameraConstants NOTE_CAM_CONSTANTS =
      new VisionIO.CameraConstants(
          "notecam", ROBOT_CENTER_TO_DETECTOR_LIMELIGHT_3D, VisionIO.CameraType.LIMELIGHT);
}
