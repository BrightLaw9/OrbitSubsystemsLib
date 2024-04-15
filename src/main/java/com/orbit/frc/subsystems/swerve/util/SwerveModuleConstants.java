package com.orbit.frc.subsystems.swerve.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int canCoderID;
    public final double angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     *
     * @param driveMotorID drive controller ID
     * @param angleMotorID angle controller ID
     * @param canCoderID ID of CanCoder on canbus
     * @param angleOffset  canCoder offset
     */

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID,
            double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.canCoderID = canCoderID;
        this.angleOffset = angleOffset;
    }

}