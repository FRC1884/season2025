package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.GlobalConstants.FieldMap;
import java.util.Optional;

/**
 * Utility functions for flipping from the blue (default) to red alliance for mirrored fields. All
 * credit goes to team 5712.
 */
public class RotationalAllianceFlipUtil {
  /**
   * Flips an x coordinate to the correct side of the field based on the current alliance color.
   *
   * @param xCoordinate The x coordinate to be flipped.
   * @return The flipped x coordinate.
   */
  public static double applyX(double xCoordinate) {
    if (shouldFlip()) {
      return FieldMap.FIELD_LENGTH_METERS - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  public static double applyY(double yCoordinate) {
    if (shouldFlip()) {
      return FieldMap.FIELD_WIDTH_METERS - yCoordinate;
    } else {
      return yCoordinate;
    }
  }

  /**
   * Flips a translation to the correct side of the field based on the current alliance color.
   *
   * @param translation The translation to be flipped.
   * @return The flipped translation.
   */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    } else {
      return translation;
    }
  }

  /**
   * Flips a 3D translation to the correct side of the field based on the current alliance color.
   *
   * @param translation The 3D translation to be flipped across the XY-plane.
   * @return The flipped 3D translation.
   */
  public static Translation3d apply(Translation3d translation) {
    if (shouldFlip()) {
      return new Translation3d(
          applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
    } else {
      return translation;
    }
  }

  /**
   * Flips a rotation based on the current alliance color.
   *
   * @param rotation The rotation to be flipped.
   * @return The flipped rotation.
   */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), -rotation.getSin());
    } else {
      return rotation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color.
   *
   * @param pose The pose to be flipped.
   * @return The flipped pose.
   */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  /**
   * Flips a 3D pose to the correct side of the field based on the current alliance color.
   *
   * @param pose The 3D pose to be flipped across the XY-plane.
   * @return The flipped 3D pose.
   */
  public static Pose3d apply(Pose3d pose) {
    if (shouldFlip()) {
      return new Pose3d(apply(pose.getTranslation()), pose.getRotation());
    } else {
      return pose;
    }
  }

  /**
   * Checks if the alliance color should be flipped.
   *
   * @return True if the alliance color should be flipped, false otherwise.
   */
  public static boolean shouldFlip() {
    Optional<Alliance> optional = DriverStation.getAlliance();
    return optional.isPresent() && optional.get() == Alliance.Red;
  }
}
