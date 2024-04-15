package com.orbit.frc.commands.swerve;

import com.orbit.frc.subsystems.swerve.SwerveSubsystem;
import com.orbit.frc.subsystems.swerve.util.PIDSwerveValues;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToPose extends Command {

    private SwerveSubsystem swerveSubsystem;
    private Pose2d target;

    private boolean allowEnd;

    /** Creates a new AlignToPose Command
     * Uses motion profiling for the translation of the swerve, and PID control for rotation based on ROBOT_ROTATION_PID in SwerveConfig
     * @param swerveSubsystem instance of swerve subsystem created
     * @param target the desired position to send robot
     * @param allowEnd if command can end; in some cases, as this command requires swerve, not allowing end prevents driver from shifting bot manually
     * The command will end if allowed to end (default is allowed), and if within provided tolerance (translation and rotation wise). 
     * Provide values in SwerveConfig 
     */
    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this(swerveSubsystem, target, true);
    }

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target, boolean allowEnd) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;
        this.allowEnd = allowEnd;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        PIDSwerveValues pidOutput = this.swerveSubsystem.calculateControlLoopDriveOutput(this.target);

        this.swerveSubsystem.drive(new Translation2d(pidOutput.xOut, pidOutput.yOut), pidOutput.rotationOut, true,
                false);
    }

    @Override
    public boolean isFinished() {
        return allowEnd && (this.swerveSubsystem.drivePIDAtTarget()
                || (Math.abs(this.swerveSubsystem.calculateDistanceToTarget(this.target)) < this.swerveSubsystem.swerveConfig.positionToleranceMeters
                        && Math.abs(this.target.getRotation().getDegrees()
                                - this.swerveSubsystem.currentPose().getRotation().getDegrees()) < this.swerveSubsystem.swerveConfig.angleToleranceDegrees));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, true, true);
    }
}
