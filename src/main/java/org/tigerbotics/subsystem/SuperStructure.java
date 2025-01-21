/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {

    private final Elevator m_elevator;
    private final Arm m_arm;

    public SuperStructure(Elevator elevator, Arm arm) {
        m_elevator = elevator;
        m_arm = arm;
    }
}
