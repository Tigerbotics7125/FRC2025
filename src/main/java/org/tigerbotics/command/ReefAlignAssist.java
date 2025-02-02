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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.tigerbotics.constant.FieldConstants;
import org.tigerbotics.constant.FieldConstants.ReefHeight;
import org.tigerbotics.constant.RobotConsts;
import org.tigerbotics.subsystem.Drivetrain;
import org.tigerbotics.subsystem.Odometry;
import org.tigerbotics.util.DriveAssist;
import org.tigerbotics.util.DriveConfig.ReefAlignAssistConfig;

@Logged
public class ReefAlignAssist {

    @NotLogged private final DriveAssist m_assist;
    @NotLogged private final Drivetrain m_drive;
    @NotLogged private final Odometry m_odom;
    @NotLogged private final ReefAlignAssistConfig m_config;

    public ReefAlignAssist(DriveAssist driveAssist) {
        m_assist = driveAssist;
        m_drive = driveAssist.getDrivetrain();
        m_odom = driveAssist.getOdometry();
        m_config = driveAssist.getConfig().reefAlignAssist();
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
                            m_odom.getField2d()
                                    .getObject("reefAlign")
                                    .setPose(
                                            m_odom.getPose2d()
                                                    .plus(
                                                            new Transform2d(
                                                                    translation,
                                                                    new Rotation2d())));
                        });

        return Commands.either(runPID, Commands.none(), assistEnabled);
    }

    public Pose2d getTargetPose() {
        // Generate all the branch postions.
        ArrayList<Pose2d> reefPoses = new ArrayList<>();
        for (int i = 0; i < FieldConstants.Reef.centerFaces.length * 2; i += 2) {
            // Right branch
            reefPoses.add(FieldConstants.Reef.branchPositions.get(i).get(ReefHeight.L1).toPose2d());
            // Left branch
            reefPoses.add(
                    FieldConstants.Reef.branchPositions.get(i + 1).get(ReefHeight.L1).toPose2d());
        }

        Pose2d closestBranch = m_odom.getPose2d().nearest(reefPoses);
        // Distance between branch and the face of the reef.
        Distance adjustX = Inches.of(2.743);
        // Go from branch pose to the robot pose for that branch.
        Transform2d branchToTargetPose =
                new Transform2d(
                        new Translation2d(
                                RobotConsts.kRobotLength.div(2).plus(adjustX), Meters.of(0)),
                        Rotation2d.k180deg);

        return closestBranch.transformBy(branchToTargetPose);
    }

    public boolean shouldAssist() {
        // First, determine if we should be assisting.
        // Calculate the vector that points from the robot to the reef.
        Vector<N2> robotToReefTrans =
                FieldConstants.Reef.center
                        .minus(m_odom.getPose2d().getTranslation())
                        .rotateBy(m_odom.getRotation2d().unaryMinus())
                        .toVector();
        // Calculate the vector that is the direction the driver intends to go.
        Vector<N2> robotTranslation =
                new Translation2d(
                                m_assist.getTranslation().getX(), -m_assist.getTranslation().getY())
                        .toVector();

        // Compute the dot between translation vector & robotToReef vector.
        double dotProduct = robotToReefTrans.unit().dot(robotTranslation.unit());

        // If the driver is not commanding the robot to move, then we still want to
        // assist.
        if (robotTranslation.equals(Translation2d.kZero.toVector())) {
            dotProduct = 1.0;
        }

        // Get the distance from the robot to the reef.
        double distanceToReef =
                FieldConstants.Reef.center.getDistance(m_odom.getPose2d().getTranslation());

        return (m_config.assistanceEnabled()
                && (dotProduct >= m_config.minDotProduct())
                && (distanceToReef <= m_config.maxDistanceToReef().in(Meters)));
    }
}
