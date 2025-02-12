/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.command;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.tigerbotics.constant.FieldConstants;
import org.tigerbotics.constant.FieldConstants.ReefHeight;
import org.tigerbotics.constant.RobotConsts;
import org.tigerbotics.subsystem.Odometry;
import org.tigerbotics.util.DriveAssistConfig.AutoAlignAssistConfig;

@Logged
public class AutoAlignAssist {

    @NotLogged private final DriveAssist m_assist;
    @NotLogged private final Odometry m_odom;
    @NotLogged private final AutoAlignAssistConfig m_config;

    @NotLogged private final ArrayList<Pose2d> targetPoses = new ArrayList<>();

    public AutoAlignAssist(DriveAssist driveAssist) {
        m_assist = driveAssist;
        m_odom = driveAssist.getOdometry();
        m_config = driveAssist.getConfig().reefAlignAssist();

        // Generate all the target postions.
        // Generate reef positions
        for (int i = 0; i < FieldConstants.Reef.centerFaces.length * 2; i += 2) {
            Pose2d branchRight =
                    FieldConstants.Reef.branchPositions.get(i).get(ReefHeight.L1).toPose2d();
            Pose2d branchLeft =
                    FieldConstants.Reef.branchPositions.get(i + 1).get(ReefHeight.L1).toPose2d();

            Translation2d branchToFace =
                    new Translation2d(Inches.of(-2.743), Inches.of(0))
                            .rotateBy(branchRight.getRotation());

            targetPoses.add(branchRight.plus(new Transform2d(branchToFace, Rotation2d.kZero)));
            targetPoses.add(branchLeft.plus(new Transform2d(branchToFace, Rotation2d.kZero)));
        }
        // Get the two stations poses
        targetPoses.add(FieldConstants.CoralStation.leftCenterFace);
        targetPoses.add(FieldConstants.CoralStation.rightCenterFace);
        // Get the processor
        targetPoses.add(FieldConstants.Processor.centerFace);

        // Send poses to field.
        m_odom.getField2d().getObject("AutoAlign Target Poses").setPoses(targetPoses);
    }

    /** @return An assistance feature Command which will help auto-align the robot to the reef. */
    public Command getCommand() {
        Supplier<Pose2d> targetPose = this::getTargetPose;
        BooleanSupplier assistEnabled = this::shouldAssist;

        PIDController translationPID = m_config.translationPID();
        PIDController rotationPID = m_config.rotationPID();

        Command runPID =
                Commands.runOnce(
                        () -> {
                            double x =
                                    translationPID.calculate(
                                            m_odom.getPose2d().getX(), targetPose.get().getX());
                            double y =
                                    translationPID.calculate(
                                            m_odom.getPose2d().getY(), targetPose.get().getY());
                            double theta =
                                    rotationPID.calculate(
                                            m_odom.getRotation2d().getDegrees(),
                                            targetPose.get().getRotation().getDegrees());

                            Translation2d translation =
                                    new Translation2d(x, -y).rotateBy(m_odom.getRotation2d());

                            m_assist.addTranslation(translation);
                            m_assist.addTheta(theta);

                            m_odom.getField2d().getObject("targetPose").setPose(targetPose.get());
                        });

        return Commands.either(runPID, Commands.none(), assistEnabled);
    }

    /** @return The closest pose which we should try to align to. */
    public Pose2d getTargetPose() {
        // Go from target pose to the robot pose for that target.
        Pose2d closestTarget = m_odom.getPose2d().nearest(targetPoses);
        Transform2d targetPoseToRobotPose =
                new Transform2d(
                        new Translation2d(RobotConsts.kRobotLength.div(2), Meters.of(0)),
                        Rotation2d.k180deg);

        return closestTarget.transformBy(targetPoseToRobotPose);
    }

    /** @return Whether the stars align and the assistance function should assist! */
    public boolean shouldAssist() {
        // First, determine if we should be assisting.
        // Calculate the vector that points from the robot to the targetPose.
        Vector<N2> robotToTargetTrans =
                getTargetPose()
                        .getTranslation()
                        .minus(m_odom.getPose2d().getTranslation())
                        .rotateBy(m_odom.getRotation2d().unaryMinus())
                        .toVector();
        // Calculate the vector that is the direction the driver intends to go.
        Vector<N2> robotTranslation =
                new Translation2d(
                                m_assist.getTranslation().getX(), -m_assist.getTranslation().getY())
                        .toVector();

        // Compute the dot between translation vector & robotToTarget vector.
        double dotProduct = robotToTargetTrans.unit().dot(robotTranslation.unit());

        // If the driver is not commanding the robot to move, then we still want to
        // assist.
        if (robotTranslation.equals(Translation2d.kZero.toVector())) {
            dotProduct = 1.0;
        }

        // Get the distance from the robot to the target.
        double distanceToPose =
                getTargetPose().getTranslation().getDistance(m_odom.getPose2d().getTranslation());

        return (m_config.assistanceEnabled()
                && (dotProduct >= m_config.minDotProduct())
                && (distanceToPose <= m_config.maxDistanceToTarget().in(Meters)));
    }
}
