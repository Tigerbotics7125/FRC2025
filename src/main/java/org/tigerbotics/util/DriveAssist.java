/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.tigerbotics.command.ReefAlignAssist;
import org.tigerbotics.subsystem.Drivetrain;
import org.tigerbotics.subsystem.Odometry;
import org.tigerbotics.util.DriveConfig.RobotOrientation;

/**
 * This class serves as the base for driving assistance commands, which are all merged into one
 * super command.
 */
@Logged
public class DriveAssist {

    @NotLogged private final Drivetrain m_drive;
    @NotLogged private final Odometry m_odom;
    private final DriveConfig m_config;

    private Translation2d m_translation = new Translation2d();
    private double m_theta = 0.0;

    public DriveAssist(Drivetrain drivetrain, Odometry odometry, DriveConfig config) {
        m_drive = drivetrain;
        m_odom = odometry;
        m_config = config;
    }

    public Drivetrain getDrivetrain() {
        return m_drive;
    }

    public Odometry getOdometry() {
        return m_odom;
    }

    public DriveConfig getConfig() {
        return m_config;
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public void addTranslation(Translation2d translation) {
        m_translation = m_translation.plus(translation);
    }

    public double getTheta() {
        return m_theta;
    }

    public void addTheta(double theta) {
        m_theta += theta;
    }

    /**
     * @param manualOverride A boolean supplier which will disable all driver assist functions when
     *     the value supplied is true.
     * @return The drive Command with driver assist features.
     */
    public Command driveAssistCmd(
            DoubleSupplier robotX,
            DoubleSupplier robotY,
            DoubleSupplier robotZ,
            BooleanSupplier manualOverride) {

        return Commands.sequence(
                        acceptHumanInputs(robotX, robotY, robotZ),
                        Commands.either(
                                Commands.none(),
                                Commands.sequence(new ReefAlignAssist(this).getCommand()),
                                manualOverride),
                        output())
                .repeatedly();
    }

    /**
     * Sends the final outputs to the drivetrain.
     *
     * @param translation The combined X & Y drive components as duty cycles (Forward + & Left +).
     * @param theta The Theta drive component as duty cycle (Counter-clockwise +).
     */
    public Command output() {
        return Commands.runOnce(
                () -> {
                    // clamp outputs.
                    double x = MathUtil.clamp(m_translation.getX(), -1, 1);
                    double y = MathUtil.clamp(m_translation.getY(), -1, 1);
                    double theta = MathUtil.clamp(m_theta, -1, 1);

                    System.out.println(x + " " + y + " " + theta);

                    m_drive.setOpenLoopSpeeds(MecanumDrive.driveCartesianIK(x, y, theta));
                },
                m_drive);
    }

    /**
     * Accepts human inputs. Note: Human inputs will override any previous inputs, and any
     * assistance should be done on top of the human inputs.
     *
     * @param robotX Duty cycle in the X direction (forwards +)
     * @param robotY Duty cycle in the Y direction (leftwards +)
     * @param robotZ Duty cycle in the Z direction (Counter-clockwise +)
     */
    public Command acceptHumanInputs(
            DoubleSupplier robotX, DoubleSupplier robotY, DoubleSupplier robotZ) {
        return Commands.runOnce(
                () -> {
                    // TODO: If we re-implement closed-loop later, just find the code in git
                    // history.
                    m_translation = new Translation2d(robotX.getAsDouble(), robotY.getAsDouble());

                    // If field oriented, then rotate the translation to match robot orientation.
                    if (m_config.robotOrientation() == RobotOrientation.kFieldOriented) {
                        m_translation = m_translation.rotateBy(m_drive.getRotation2d());
                    }

                    m_theta = robotZ.getAsDouble();
                });
    }
}
