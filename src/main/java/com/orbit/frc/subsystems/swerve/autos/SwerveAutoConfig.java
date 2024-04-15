package com.orbit.frc.subsystems.swerve.autos;

import java.util.Optional;

import com.orbit.frc.subsystems.swerve.SwerveConfig;
import com.orbit.frc.subsystems.swerve.SwerveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class SwerveAutoConfig {

    public static void configureAutoBuilder(SwerveSubsystem swerve, SwerveConfig swerveConfig) {
        AutoBuilder.configureHolonomic(
                swerve::currentPose,
                swerve::setCurrentPose,
                swerve::getRobotRelativeSpeeds,
                swerve::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        swerveConfig.translation,
                        swerveConfig.rotation,
                        swerveConfig.MAX_SPEED,
                        Math.sqrt(Math.pow(swerveConfig.TRACK_WIDTH_METERS / 2, 2)
                                + Math.pow(swerveConfig.WHEEL_BASE_METERS / 2, 2)), // Pythagorean distance for robot
                                                                                 // center to a module
                        new ReplanningConfig()),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    SmartDashboard.putBoolean("AutoBuilder isBlue: ",
                            (alliance.get() != null && alliance.get() == DriverStation.Alliance.Blue));
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false; // Defaults to no path flipping if no alliance data available
                },
                swerve);
        
        Pathfinding.setPathfinder(new LocalADStar());
        System.out.println("AutoBuilder Configured!");
    }
}
