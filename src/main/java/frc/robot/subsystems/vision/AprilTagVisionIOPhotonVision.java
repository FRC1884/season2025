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

package frc.robot.subsystems.vision;

import static frc.robot.GlobalConstants.FieldMap.APRIL_TAG_FIELD_LAYOUT;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.*;
import lombok.Getter;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * IO implementation for a real PhotonVision camera running an AprilTag pipeline on the coprocessor.
 * PhotonVision also publishes the angles to the best tag in view, which can be helpful for tag
 * tracking and alignment.
 */
public class AprilTagVisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera; // We want the camera to be available in the sim wrapper too
  @Getter private final CameraConstants cameraConstants;

  /**
   * Creates a new AprilTagVisionIOPhotonVision.
   *
   * @param cameraConstants The constants associated with this camera.
   */
  public AprilTagVisionIOPhotonVision(CameraConstants cameraConstants) {
    this.cameraConstants = cameraConstants;
    camera = new PhotonCamera(cameraConstants.cameraName());
  }

  /**
   * Updates and processes all inputs this frame, including detected tags and estimated poses.
   *
   * @param inputs the object to store updated vision data.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();

    var pipelineResults = camera.getAllUnreadResults(); // Call this ONCE per robot loop!

    for (PhotonPipelineResult result : pipelineResults) {
      // Update latest target observation (not for pose estimation)
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            // Remember to set the target sort mode in PV UI to select which tag you want to get
            // (e.g. closest, least ambiguity)
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        // Nothing seen, publish empty observation
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      if (result.multitagResult.isPresent()) {
        // Process multi-tag estimations
        MultiTargetPNPResult multitagResult = result.multitagResult.get();

        // Calculate field-relative robot pose using PnP on all tags
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(cameraConstants.robotToCamera().inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (PhotonTrackedTarget target : result.targets) {
          double distanceToTarget;
          if ((distanceToTarget = target.bestCameraToTarget.getTranslation().getNorm())
              < cameraConstants.cameraType().noisyDistance) {
            totalTagDistance += distanceToTarget;
            // Add detected tag IDs
            tagIds.add((short) target.fiducialId);
          }
        }

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                tagIds.size(), // Tag count
                totalTagDistance / tagIds.size() // Average tag distance
                ));

      } else if (!result.targets.isEmpty()) {
        // Process single-tag estimations
        PhotonTrackedTarget target = result.targets.get(0);
        Optional<Pose3d> tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);

        if (tagPose.isPresent()) {
          // Calculate field-relative robot pose using a single tag
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(cameraConstants.robotToCamera().inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add detected tag ID
          if (cameraToTarget.getTranslation().getNorm()
              < cameraConstants.cameraType().noisyDistance) {
            tagIds.add((short) target.fiducialId);

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose, // 3D pose estimate
                    target.poseAmbiguity, // Ambiguity
                    1, // One tag seen
                    cameraToTarget.getTranslation().getNorm() // Average tag distance
                    ));
          }
        }
      }
    }

    // Save pose observations to inputs
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
