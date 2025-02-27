/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmConsts {

    public static final int kMotorID = 21;

    public static final Alert kConfigAlert =
            new Alert(
                    "Arm: SparkMAX configure failed! (Performance may be affected!)",
                    AlertType.kError);

    // TODO: These were tuned for sim, obviously will be different IRL.
    public static final ProfiledPIDController kPIDController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(45, 90));

    // TODO: Figure out the actual gear ratio
    public static final Dimensionless kGearRatio = Rotations.of(100).div(Rotations.of(1));

    public static final SparkMaxConfig kConfig = new SparkMaxConfig();

    static {
        // Create a new config object.
        SparkMaxConfig config = new SparkMaxConfig();

        // Limits the max current draw.
        config.smartCurrentLimit(60);

        // Set the conversion factor to convert from motor roations / RPM to arm degrees / degrees
        // per second.
        config.encoder.positionConversionFactor(Rotations.of(1).times(kGearRatio).in(Degrees));
        config.encoder.velocityConversionFactor(RPM.of(1).times(kGearRatio).in(DegreesPerSecond));

        // Update the constant config object.
        kConfig.apply(config);
    }

    public static final SingleJointedArmSim kArmSim =
            new SingleJointedArmSim(
                    DCMotor.getNEO(1),
                    kGearRatio.magnitude(),
                    .5,
                    Units.feetToMeters(1),
                    Units.degreesToRadians(-360),
                    Units.degreesToRadians(360),
                    true,
                    0);
}
