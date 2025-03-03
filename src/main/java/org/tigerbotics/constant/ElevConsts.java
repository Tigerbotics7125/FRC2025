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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevConsts {

    public static final int kLeftID = 11;
    public static final int kRightID = 12;

    public static final Alert kConfigAlert =
            new Alert(
                    "Elevator: SparkMAX configure failed! (Performance may be affected!)",
                    AlertType.kError);

    // TODO: These were tuned for sim, obviously will need to be re-tuned IRL.
    public static final ProfiledPIDController kPIDController =
            new ProfiledPIDController(100, 0, 0, new TrapezoidProfile.Constraints(2, 1.0));

    // TODO: Figure out the actual gear ratio
    public static final Dimensionless kGearRatio = Rotations.of(36).div(Rotations.of(1));
    // TODO: Check this number (currently 15/16", which is closest fraction to 2023 value.)
    public static final Distance kSprocketCircumference =
            Value.of(2 * Math.PI).times(Inches.of(0.9375));
    // TODO: Check this number, taken from 2023 code, should be the same.
    public static final Dimensionless kPullyRatio = Value.of(2);
    public static final Per<DistanceUnit, AngleUnit> kPositionConversionFactor =
            Value.of(1)
                    .div(kGearRatio)
                    .times(kSprocketCircumference)
                    .times(kPullyRatio)
                    .div(Rotation.of(1));
    public static final Per<LinearVelocityUnit, AngularVelocityUnit> kVelocityConversionFactor =
            Value.of(1)
                    .div(kGearRatio)
                    .times(kSprocketCircumference)
                    .times(kPullyRatio)
                    .per(Minute)
                    .div(RPM.of(1));

    public static final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightConfig = new SparkMaxConfig();

    static {
        // Create a new config object.
        SparkMaxConfig config = new SparkMaxConfig();

        // Limit the max current draw.
        config.smartCurrentLimit(60);

        // Set the conversion factor to convert from motor rotations / RPM to elevator meters /
        // meters per second.
        config.encoder.positionConversionFactor(
                Rotation.of(1).timesConversionFactor(kPositionConversionFactor).in(Meters));
        config.encoder.velocityConversionFactor(
                RPM.of(1).timesConversionFactor(kVelocityConversionFactor).in(MetersPerSecond));

        // Apply the config to the two motor configs.
        kLeftConfig.apply(config);
        kRightConfig.apply(config);
        // Make sure the right motor follows the left.
        kRightConfig.follow(kLeftID, true);
    }

    // TODO: I have no idea if these are real or not, should only matter for sim I think though.
    public static final Distance kMinHeight = Feet.of(2.5);
    public static final Distance kMaxHeight = Feet.of(6);

    public static final ElevatorSim kElevSim =
            new ElevatorSim(
                    LinearSystemId.createElevatorSystem(
                            DCMotor.getNEO(2), 20, Units.inchesToMeters(0.9375), 36),
                    DCMotor.getNEO(2),
                    kMinHeight.in(Meters),
                    kMaxHeight.in(Meters),
                    true,
                    Units.feetToMeters(3));
}
