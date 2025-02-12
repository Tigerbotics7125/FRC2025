/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.util;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;

/**
 * I know this should probably just be a constant file but I wanted it to be able to be changed on
 * the fly, so I need it mutable.
 */
@Logged
public class DriveAssistConfig {
    /** Defines which direction the robot assumes forwards to be. */
    public enum RobotOrientation {
        // Assumes that forward is away from the driver station.
        kFieldOriented,
        // Assumes that forward is towards the front of the robot.
        kRobotOriented;
    }

    @Logged
    public class AutoAlignAssistConfig {
        private boolean assistanceEnabled = true;
        private double minDotProduct = 0.15;
        private Distance maxDistanceToTarget = Feet.of(3);
        private PIDController translationPID = new PIDController(2.0, 0.0, 0.0);
        private PIDController rotationPID = new PIDController(0.01, 0.0, 0.0);

        // Make sure default PID is continuous.
        {
            rotationPID.enableContinuousInput(0, 360);
        }

        /** @param assistanceEnabled Whether to enable this assistance or not. */
        public AutoAlignAssistConfig withAssistanceEnabled(boolean assistanceEnabled) {
            this.assistanceEnabled = assistanceEnabled;
            return this;
        }

        public boolean assistanceEnabled() {
            return assistanceEnabled;
        }

        /**
         * @param minDotProduct The minimum dot product value to ensure the driver is intending to
         *     go towards the reef.
         */
        public AutoAlignAssistConfig withMinDotProduct(double minDotProduct) {
            this.minDotProduct = minDotProduct;
            return this;
        }

        public double minDotProduct() {
            return minDotProduct;
        }

        /**
         * @param minDistanceToTarget The robot must be this close to the target before the
         *     assistance will enable.
         */
        public AutoAlignAssistConfig withMaxDistanceToTarget(Distance minDistanceToTarget) {
            this.maxDistanceToTarget = minDistanceToTarget;
            return this;
        }

        public Distance maxDistanceToTarget() {
            return maxDistanceToTarget;
        }

        /** @param translationPID The translational PIDController. */
        public AutoAlignAssistConfig withTranslationPID(PIDController translationPID) {
            this.translationPID = translationPID;
            return this;
        }

        public PIDController translationPID() {
            return translationPID;
        }

        /** @param rotationPID The rotational PIDController. */
        public AutoAlignAssistConfig withRotationPID(PIDController rotationPID) {
            this.rotationPID = rotationPID;
            rotationPID.enableContinuousInput(0, 360);
            return this;
        }

        public PIDController rotationPID() {
            return rotationPID;
        }
    }

    // Defines how the controllers forward direction should be interpreted by the drivetrain.
    private RobotOrientation robotOrientation = RobotOrientation.kRobotOriented;
    // Configurations related to the reef align assistance feature.
    private AutoAlignAssistConfig reefAlignAssist = new AutoAlignAssistConfig();

    public DriveAssistConfig withRobotOrientation(RobotOrientation robotOrientation) {
        this.robotOrientation = robotOrientation;
        return this;
    }

    public RobotOrientation robotOrientation() {
        return robotOrientation;
    }

    public DriveAssistConfig withReefAlignAssist(AutoAlignAssistConfig reefAlignAssist) {
        this.reefAlignAssist = reefAlignAssist;
        return this;
    }

    public AutoAlignAssistConfig reefAlignAssist() {
        return reefAlignAssist;
    }
}
