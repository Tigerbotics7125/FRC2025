/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    SparkMax m_Intake_Left = new SparkMax(0, MotorType.kBrushless);
    SparkMax m_Intake_Right = new SparkMax(0, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    public static final SparkMaxConfig m_Intake_Left_Config = new SparkMaxConfig();
    public static final SparkMaxConfig m_Intake_Right_Config = new SparkMaxConfig();

    {
        config.smartCurrentLimit(80);

        // Set the conversion factors, converting motor position and velocity (rotations
        // & RPM) to linear wheel position & valocity (meters & meters/second)

        // Set the config objects.
        m_Intake_Left_Config.apply(config);
        m_Intake_Right_Config.apply(config);
        m_Intake_Right_Config.follow(m_Intake_Left);

        // Invert the right side. (This must be done after the apply otherwise it would
        // get overwritten.)
        m_Intake_Left_Config.inverted(false);
        m_Intake_Right_Config.inverted(true);
        m_Intake_Left.configure(
                m_Intake_Left_Config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command disable() {
        return run(m_Intake_Left::disable);
    }

    public Command intake() {
        return run(() -> m_Intake_Left.set(0.25));
    }
}
