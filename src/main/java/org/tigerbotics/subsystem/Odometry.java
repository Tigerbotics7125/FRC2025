/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tigerbotics.Robot;
import org.tigerbotics.constant.DriveConsts;

@Logged
public class Odometry extends SubsystemBase {

    private Drivetrain m_drivetrain;
    private Vision m_vision;

    private final MecanumDrivePoseEstimator m_poseEstimator;
    // Odometry using only robot sensors for sim.
    private final MecanumDriveOdometry m_simOdometry;

    // Field for simulation
    private final Field2d m_field = new Field2d();

    public Odometry(Drivetrain drivetrain, Vision vision) {
        m_drivetrain = drivetrain;
        m_vision = vision;

        m_poseEstimator =
                new MecanumDrivePoseEstimator(
                        DriveConsts.kKinematics,
                        m_drivetrain.getRotation2d(),
                        m_drivetrain.getWheelPositions(),
                        new Pose2d(),
                        // TODO: these will need to be tuned better, but currently trust vision a
                        // little more than wheels.
                        VecBuilder.fill(0.3, 0.3, 0.1),
                        VecBuilder.fill(0.15, 0.15, 0.15));

        m_simOdometry =
                new MecanumDriveOdometry(
                        DriveConsts.kKinematics,
                        m_drivetrain.getRotation2d(),
                        m_drivetrain.getWheelPositions(),
                        new Pose2d());
    }

    @Override
    public void periodic() {
        // Update the pose estimator with current drivetrain values.
        m_poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                m_drivetrain.getRotation2d(),
                m_drivetrain.getWheelPositions());

        /**
         * This is just a really fancy way to iterate through the list of estimated poses, check if
         * it is present, if so, add it to the pose estimator, otherwise do nothing.
         */
        m_vision.getEstimatedPoses()
                .forEach(
                        (potentialVisionPose) ->
                                potentialVisionPose.ifPresentOrElse(
                                        (visionPose) -> {
                                            Pose2d estimatedPose =
                                                    visionPose.estimatedPose.toPose2d();
                                            double distanceFromCurrentPose =
                                                    estimatedPose
                                                            .minus(getPose2d())
                                                            .getTranslation()
                                                            .toVector()
                                                            .norm();

                                            // TODO: this can go bad if the pose ever gets > 0.5m
                                            // off
                                            // Perhaps a running average of normals of the vision
                                            // estimation so if they're close still add them.

                                            // Only add the pose if it is within 0.5m of the current
                                            // pose estimation.
                                            if (distanceFromCurrentPose < 0.5) {

                                                m_poseEstimator.addVisionMeasurement(
                                                        visionPose.estimatedPose.toPose2d(),
                                                        visionPose.timestampSeconds);
                                            }
                                        },
                                        () -> {}));

        // Set the robot position on the field (for dashboard / sim).
        m_field.setRobotPose(getPose2d());
        SmartDashboard.putData(m_field);
    }

    @Override
    public void simulationPeriodic() {
        m_simOdometry.update(m_drivetrain.getRotation2d(), m_drivetrain.getWheelPositions());
        m_vision.updateVisionSim(m_simOdometry.getPoseMeters());

        m_field.getObject("Real Pose").setPose(m_simOdometry.getPoseMeters());
    }

    /** @return The current best estimation of the robot's position. */
    public Pose2d getPose2d() {
        if (Robot.isReal()) {
            return m_poseEstimator.getEstimatedPosition();
        } else {
            return m_simOdometry.getPoseMeters();
        }
    }

    /** @return The current best estimation of the robot's rotation. */
    public Rotation2d getRotation2d() {
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Resets the pose estimator with the provided pose.
     *
     * @param currentPose The new current pose.
     */
    public void setPose2d(Pose2d currentPose) {
        m_poseEstimator.resetPose(currentPose);
        m_simOdometry.resetPose(currentPose);
    }

    public Field2d getField2d() {
        return m_field;
    }
}
