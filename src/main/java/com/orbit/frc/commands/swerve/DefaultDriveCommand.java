package com.orbit.frc.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import com.orbit.frc.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {
    private final SwerveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final CommandJoystick rotationJoystick;

    private boolean pressed = false;

    /**
     * Constructs a field-oriented teleop drive command that takes in an axis for:  
     * @param translationXSupplier x-axis movement (towards (-) and away (+) from alliance walls)
     * @param translationYSupplier y-axis movement (to left wall (+) and right scoring table wall (-) based on blue alliance) 
     * @param rotationSupplier rotational movement (positive joystick input is rotating CW)
     */
    public DefaultDriveCommand(SwerveSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            CommandJoystick rotationJoystick) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.rotationJoystick = rotationJoystick;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement

        m_drivetrainSubsystem.drive(
                new Translation2d(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble()),
                -m_rotationSupplier.getAsDouble(),
                true, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, true, true);
    }

    private double getDirection(double target, double current) {
        if (target == current) {
            return 0;
        }
        if (Math.abs(current - target) < 180.0) {
            if (current < target) {
                return 1.0;
            } else {
                return -1.0;
            }
        } else {
            if (current < target) {
                return -1.0;
            } else {
                return 1.0;
            }
        }
    }
}
