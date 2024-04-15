package com.orbit.frc.subsystems.swerve;

import com.orbit.frc.subsystems.SubsystemConfig;
import com.orbit.frc.subsystems.SubsystemConfig.MotorModel;
import com.orbit.frc.subsystems.swerve.util.PIDConstants;
import com.orbit.frc.subsystems.swerve.util.SwerveModuleConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

public class SwerveConfig {
    
    /** if gyro is inverted, going CCW is considered positive (adding) degrees - WPILib poses uses gyro invert */
    public boolean isGyroInverted;
    public static MotorModel driveMotorType; 
    public static MotorModel angleMotorType; 

    /** moduleConstants A array of length 4 or how ever many swerve modules are being used should be provided  
     * Standard convention: 0th index is FL, 1st index is FR, 2nd index is BL, 3rd index is BR 
    */
    public SwerveModuleConstants[] moduleConstants; 

    public CANSparkBase[] driveMotors; 
    public CANSparkBase[] angleMotors; 

    /** The distance between the centerline of the wheels on the same "axle" (for example, distance between FL and FR modules) */
    public double TRACK_WIDTH_METERS; 
    /** The distance between front and back wheels (according to there centers) */
    public double WHEEL_BASE_METERS; 

    /** Optional: available for customization; created based on TRACK_WIDTH and WHEEL_BASE by default */
    public SwerveDriveKinematics swerveDriveKinematics; 

    public double WHEEL_DIAMETER_METERS; 
    public double DRIVE_GEAR_RATIO; 
    public double ANGLE_GEAR_RATIO; 

    protected double DRIVE_CONVERSION_POSITION_FACTOR; 
    protected double DRIVE_CONVERSION_VELOCITY_FACTOR;
    protected double ANGLE_CONVERSION_FACTOR;

    /** The current limit for when the motor is undergoing acceleration/force is being applied */
    public int STALL_CURRENT_LIMIT;
    /** Motor is spinning freely at a constant speed */
    public int FREE_CURRENT_LIMIT; 

    /** Is the way swerve modules rotate inverted? */
    public boolean ANGLE_INVERT; 
    /** Is the way swerve modules drive spin inverted? */
    public boolean DRIVE_INVERT; 
    /** idle mode (brake or coast) for drive and angle motors */
    public IdleMode IDLE_MODE; 

    /** PID for drive - module speeds following their target speeds */
    public PIDConstants DRIVE_PID; 
    /** IMPORTANT: TUNE ANGLE_PID to be precise - modules need to be pointing accurately at their target angles */
    public PIDConstants ANGLE_PID; 
    /** <p> 
     * Feedforward for drive
     * kv is calculated = optimal Voltage / maxSpeed; 
     * ka = optimal voltage / maxAcceleration (practically is the coefficient of friction * 9.81 causing accel)
     * </p>
    */
    public SimpleMotorFeedforward DRIVE_SVA; 

    /** PID that controls the alignment of the robot (rotation wise, to a target) */
    public PIDConstants ROBOT_ROTATION_PID; 

    /** MAX speed for robot travel */
    public double MAX_SPEED; 
    public double MAX_ACCELERATION; 

    // Auto Constants
    public com.pathplanner.lib.util.PIDConstants translation; 
    public com.pathplanner.lib.util.PIDConstants rotation; 
    public double maxAngularVelocityRadians; 
    public double maxAngularAccelerationRadians; 
    public double positionToleranceMeters; 
    public double angleToleranceDegrees; 
    

    /**
     * This method should be used to initalize SwerveSubsystem; 
     */
    public static SwerveSubsystem createSwerve(SwerveConfig config, VisionConfig visionConfig) { 

        // set up the motors
        for(int i = 0; i < config.moduleConstants.length; i++) {
            switch(driveMotorType) {
                // add krakens later on? 
                case REV_NEO:
                    config.driveMotors[i] = new CANSparkMax(config.moduleConstants[i].driveMotorID, MotorType.kBrushless);
                    break;
                case REV_NEO_550:
                    config.driveMotors[i] = new CANSparkMax(config.moduleConstants[i].driveMotorID, MotorType.kBrushless);
                    break;
                case REV_VORTEX:
                    config.driveMotors[i] = new CANSparkFlex(config.moduleConstants[i].driveMotorID, MotorType.kBrushless);
                    break;
                default: 
                    break; 
            }
            
            switch(angleMotorType) {
                case REV_NEO:
                    config.angleMotors[i] = new CANSparkMax(config.moduleConstants[i].angleMotorID, MotorType.kBrushless);
                    break;
                case REV_NEO_550:
                    config.angleMotors[i] = new CANSparkMax(config.moduleConstants[i].angleMotorID, MotorType.kBrushless);
                    break;
                case REV_VORTEX:
                    config.angleMotors[i] = new CANSparkFlex(config.moduleConstants[i].angleMotorID, MotorType.kBrushless);
                    break;
                default: 
                    break; 
            }
        }

        if (config.swerveDriveKinematics == null) { 
                config.swerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(config.WHEEL_BASE_METERS / 2.0, config.TRACK_WIDTH_METERS / 2.0),
                new Translation2d(config.WHEEL_BASE_METERS / 2.0, -config.TRACK_WIDTH_METERS / 2.0),
                new Translation2d(-config.WHEEL_BASE_METERS / 2.0, config.TRACK_WIDTH_METERS / 2.0),
                new Translation2d(-config.WHEEL_BASE_METERS / 2.0, -config.TRACK_WIDTH_METERS / 2.0));
        }
        config.DRIVE_CONVERSION_POSITION_FACTOR = (config.WHEEL_DIAMETER_METERS * Math.PI) / config.DRIVE_GEAR_RATIO;
        config.DRIVE_CONVERSION_VELOCITY_FACTOR = config.DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
        config.ANGLE_CONVERSION_FACTOR = 360.0 / config.ANGLE_GEAR_RATIO;
        
        return new SwerveSubsystem(config, visionConfig); 
    }
}
