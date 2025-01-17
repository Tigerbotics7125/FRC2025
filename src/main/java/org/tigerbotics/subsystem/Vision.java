/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static org.tigerbotics.constant.VisionConsts.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

    // The camera.
    private final PhotonCamera m_camera = new PhotonCamera(kCameraName);
    private final PhotonCameraSim m_cameraSim = new PhotonCameraSim(m_camera, kCameraProps);

    // The pose estimator to process data.
    private final PhotonPoseEstimator m_poseEstimator =
            new PhotonPoseEstimator(kFieldTags, kPoseStrategy, kRobotToCamera);

    // Simulated vision environment for simulating cameras.
    private final VisionSystemSim m_visionSim = new VisionSystemSim("main");

    public Vision() {
        // Alert if camera not connected.
        kCameraNotConnected.set(!m_camera.isConnected());

        // Make sure we're using the camera for robot vision not human vision.
        m_camera.setDriverMode(false);

        // Whenever we only see one tag, use the position
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Add the april tags to the vision environment.
        m_visionSim.addAprilTags(kFieldTags);
        // Add the camera to the vision environment.
        m_visionSim.addCamera(m_cameraSim, kRobotToCamera);
    }

    public List<Optional<EstimatedRobotPose>> getEstimatedPoses() {
        /**
         * It's possible to have multiple camera results per robot loop, especially if the camera is
         * updating quickly (say 60fps) 60fps * 0.02s (loop period) = 1.2 frames so ocassionaly
         * frames may be missed per robot loop if we do not account for the chance of multiple
         * frames per loop.
         *
         * <p>This is exacerbated if the robot loop hangs, vision results will accumulate while the
         * robot does its thing.
         */

        // Get all of the camera results in the queue.
        List<PhotonPipelineResult> cameraResults = m_camera.getAllUnreadResults();

        List<Optional<EstimatedRobotPose>> visionPoses = new ArrayList<>();

        for (PhotonPipelineResult cameraResult : cameraResults) {
            visionPoses.add(m_poseEstimator.update(cameraResult));
        }

        return visionPoses;
    }

    /**
     * Update the simulated vision system.
     *
     * @param robotPose The current robot position.
     */
    public void updateVisionSim(Pose2d robotPose) {
        m_visionSim.update(robotPose);
    }
}
