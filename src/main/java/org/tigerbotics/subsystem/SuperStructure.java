/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class SuperStructure extends SubsystemBase {

    private final Elevator m_elevator;
    private final Arm m_arm;

    private final Mechanism2d m_mech = new Mechanism2d(1, 1);
    private final MechanismRoot2d m_root = m_mech.getRoot("superstruc", 0.5, 0);
    private final MechanismLigament2d m_elevMech =
            m_root.append(new MechanismLigament2d("elevator", 0.25, 90));
    private final MechanismLigament2d m_armMech =
            m_elevMech.append(new MechanismLigament2d("arm", .25, -90));

    public SuperStructure(Elevator elevator, Arm arm) {
        m_elevator = elevator;
        m_arm = arm;

        SmartDashboard.putData("SuperStructure Mechanism", m_mech);
    }

    @Override
    public void simulationPeriodic() {
        m_armMech.setAngle(m_arm.getPosition().in(Degrees) - m_elevMech.getAngle());
    }
}
