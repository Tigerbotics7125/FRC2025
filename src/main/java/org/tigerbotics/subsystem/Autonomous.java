/*
 * Copyright (c) 2025 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package org.tigerbotics.subsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tigerbotics.constant.RobotConsts;

public class Autonomous extends SubsystemBase {
    Drivetrain drive;
    Odometry odom;

    public Autonomous(Drivetrain drive, Odometry odom) {
        this.drive = drive;
        this.odom = odom;
        // All other subsystem initialization
        NamedCommands.registerCommand("print", Commands.print("asdf"));

        // Configure AutoBuilder last
        AutoBuilder.configure(
                odom::getPose2d, // Robot pose supplier
                odom::setPose2d, // Method to reset odometry (will be called if your auto has a
                // starting pose)
                drive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) ->
                        drive.setTargetChassisSpeeds(
                                speeds), // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds. Also optionally outputs individual module
                // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                        // following controller for holonomic drive trains
                        new PIDConstants(25, 0.0, 0), // Translation PID constants
                        new PIDConstants(0, 0.0, 0.0) // Rotation PID constants
                        ),
                RobotConsts.kRobotConfig, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
                );
    }

    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return new PathPlannerAuto("Example Auto");
    }
}
