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
        START(Meters.of(0), Degrees.of(0)),
        DEMO(Meters.of(1), Degrees.of(0)),
        GROUND(Meters.of(0), Degrees.of(0)),
        FIRST(Meters.of(.5), Degrees.of(0)),
        SECOND(Meters.of(1), Degrees.of(0)),
        THIRD(Meters.of(1.25), Degrees.of(0)),
        FOURTH(Meters.of(1.5), Degrees.of(0));

        public final Distance elevDistance;
        public final Angle armAngle;

        SuperStructState(Distance elevDistance, Angle armAngle) {
            this.elevDistance = elevDistance;
            this.armAngle = armAngle;
        }
    }
}
