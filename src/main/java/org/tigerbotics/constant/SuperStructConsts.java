/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.constant;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SuperStructConsts {

    public enum SuperStructState {
        START(Meters.of(0), Degrees.of(-90)),
        DEMO(Meters.of(1), Degrees.of(30));

        public final Distance elevDistance;
        public final Angle armAngle;

        SuperStructState(Distance elevDistance, Angle armAngle) {
            this.elevDistance = elevDistance;
            this.armAngle = armAngle;
        }
    }
}
