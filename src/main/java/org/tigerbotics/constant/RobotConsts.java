/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class RobotConsts {
    public static final Distance kRobotLengthNoBumpers = Inches.of(32.3);
    public static final Distance kRobotWidthNoBumpers = Inches.of(27);
    public static final Distance kBumperThickness = Inches.of(3);
    public static final Distance kRobotLength = kRobotLengthNoBumpers.plus(kBumperThickness);
    public static final Distance kRobotWidth = kRobotWidthNoBumpers.plus(kBumperThickness);

    public static final Mass kRobotMass = Pound.of(120);
    public static final MomentOfInertia kRobotMoI =
            KilogramSquareMeters.of(4.0); // ChatGPT says this is a reasonable guess. :shrug:

    public static final RobotConfig kRobotConfig =
            new RobotConfig(
                    kRobotMass,
                    kRobotMoI,
                    new ModuleConfig(
                            DriveConsts.kWheelRadius,
                            DriveConsts.kMaxLinearVelocity,
                            1.0,
                            DCMotor.getNEO(1),
                            DriveConsts.kGearRatio.magnitude(),
                            Amps.of(80),
                            1),
                    DriveConsts.kFLTranslation,
                    DriveConsts.kFRTranslation,
                    DriveConsts.kRLTranslation,
                    DriveConsts.kRRTranslation);
}
