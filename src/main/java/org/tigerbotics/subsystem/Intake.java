/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static org.tigerbotics.constant.IntakeConsts.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkMax m_left = new SparkMax(kLeftID, kMotorType);
    private final SparkMax m_right = new SparkMax(kRightID, kMotorType);

    public Intake() {
        m_left.configure(
                kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_right.configure(
                kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** @return A Command which disables motor output. */
    public Command disable() {
        return run(m_left::disable);
    }

    /** @return A Command which intakes. */
    public Command intake() {
        return runOnce(() -> m_left.set(kIntakeSpeed));
    }

    /** @return A Command which outtakes. */
    public Command outtake() {
        return runOnce(() -> m_left.set(kOuttakeSpeed));
    }

    // TODO: Look into automating intake sequences with current draw monitoring.
}
