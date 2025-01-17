/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConsts {

    // The name of the camera as seen in the photon dashboard.
    public static final String kCameraName = "OV9281";

    // The april tag layout (where each tag is in 3d space).
    public static final AprilTagFieldLayout kFieldTags =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // How the pose estimator should estimate our pose.
    public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    // TODO: Get this value.
    // How to transform from the robot pose to camera pose.
    public static final Transform3d kRobotToCamera = new Transform3d();

    // Disconnection alert.
    public static final Alert kCameraNotConnected =
            new Alert("Vision: Camera not connected! (Odometry will suffer!)", AlertType.kError);

    // Properties of our camera, used to simulate it properly.
    public static final SimCameraProperties kCameraProps = new SimCameraProperties();

    // Innomaker OV9281 USB
    static {
        kCameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(148));
        kCameraProps.setCalibError(0.25, 0.08);
        kCameraProps.setFPS(62);
        kCameraProps.setAvgLatencyMs(14);
        kCameraProps.setLatencyStdDevMs(5);
    }
}
