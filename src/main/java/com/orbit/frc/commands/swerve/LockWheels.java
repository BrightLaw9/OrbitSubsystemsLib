package com.orbit.frc.commands.swerve;

import com.orbit.frc.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class LockWheels extends Command {

    SwerveSubsystem swerveSubsystem;

    public LockWheels(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.brake();
    }

}
