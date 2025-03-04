/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConsts {

    public static final double kIntakeSpeed = 0.75;
    public static final double kOuttakeSpeed = -0.5;
    public static final double kKickSpeed = 0.5;

    public static final int kLeftID = 9;
    public static final int kRightID = 8;

    // We're using NEOs
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final SparkMaxConfig kLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kRightConfig = new SparkMaxConfig();

    static {
        // Setup the shared config object.
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20);

        // Apply the config to all configurations.
        kLeftConfig.apply(config);
        kRightConfig.apply(config);

        // Setup motor independent changes.
        // kRightConfig.follow(kLeftID, true);
    }
}
