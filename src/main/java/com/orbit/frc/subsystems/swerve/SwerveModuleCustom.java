package com.orbit.frc.subsystems.swerve;

import org.littletonrobotics.junction.AutoLogOutput;

import com.orbit.frc.subsystems.swerve.util.CANSparkMaxUtil;
import com.orbit.frc.subsystems.swerve.util.PIDConstants;
import com.orbit.frc.subsystems.swerve.util.SwerveModuleConstants;
import com.orbit.frc.subsystems.swerve.util.CANSparkMaxUtil.Usage;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.orbit.frc.subsystems.swerve.math.OnboardModuleState;
import com.orbit.frc.subsystems.swerve.encoders.CANCoderSwerve; 

public class SwerveModuleCustom {
    /* Module details */
    public int moduleNumber;

    /* Motors */
    private CANSparkBase angleMotor;
    private CANSparkBase driveMotor;

    /* Encoders and their values */
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    // public MagEncoder angleEncoder;
    public CANCoderSwerve angleEncoder;

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/LastAngle")
    private double lastAngle;

    /* Controllers */
    public final SparkPIDController driveController;
    public final SparkPIDController angleController;
    public SimpleMotorFeedforward feedforward;

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/TargetSpeed")
    public double targetSpeed = 0;

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/TargetAngle")
    public double targetAngle = 0;

    private SwerveModuleState targetState = new SwerveModuleState();

    private double currentAcceleration;
    private double prevSpeed = 0.0;
    private double prevTime = -1;

    private SwerveConfig swerveConfig; 

    public SwerveModuleCustom(int moduleNumber, SwerveModuleConstants moduleConstants, SwerveConfig swerveConfig) {
        this.moduleNumber = moduleNumber;
        this.swerveConfig = swerveConfig; 

        /* Angle Encoder Config */
        angleEncoder = new CANCoderSwerve(moduleConstants.canCoderID);

        System.out.println(angleEncoder.setAbsoluteEncoderOffset(moduleConstants.angleOffset));

        /* Angle Motor Config */
        this.angleMotor = swerveConfig.angleMotors[moduleNumber];
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        this.driveMotor = swerveConfig.driveMotors[moduleNumber];
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = OnboardModuleState.optimize(
                desiredState,
                getState().angle); // Custom optimize command, since default WPILib optimize assumes
        // continuous controller which REV and CTRE are not

        this.setSpeed(desiredState, isOpenLoop);
        this.setAngle(desiredState);
        this.targetState = desiredState;
    }

    public SwerveModuleState getDesiredState() {
        return targetState;
    }

    public void resetToAbsolute() {
        integratedAngleEncoder.setPosition(this.getCanCoder().getDegrees());
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(swerveConfig.FREE_CURRENT_LIMIT);
        angleMotor.setInverted(swerveConfig.ANGLE_INVERT);
        angleMotor.setIdleMode(swerveConfig.IDLE_MODE);
        integratedAngleEncoder.setPositionConversionFactor(swerveConfig.ANGLE_CONVERSION_FACTOR);
        swerveConfig.ANGLE_PID.applyPID(this.angleController);
        angleController.setFF(0);
        angleMotor.enableVoltageCompensation(12.0);
        this.resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(swerveConfig.STALL_CURRENT_LIMIT, swerveConfig.FREE_CURRENT_LIMIT);
        driveMotor.setInverted(swerveConfig.DRIVE_INVERT);
        driveMotor.setIdleMode(swerveConfig.IDLE_MODE);
        driveEncoder.setVelocityConversionFactor(swerveConfig.DRIVE_CONVERSION_VELOCITY_FACTOR);
        driveEncoder.setPositionConversionFactor(swerveConfig.DRIVE_CONVERSION_POSITION_FACTOR);
        swerveConfig.DRIVE_PID.applyPID(this.driveController);
        driveController.setFF(0);
        this.feedforward = swerveConfig.DRIVE_SVA;
        driveMotor.enableVoltageCompensation(12.0);
        driveEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / swerveConfig.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            this.targetSpeed = desiredState.speedMetersPerSecond;
            if (this.prevTime == -1) {
                this.prevTime = System.currentTimeMillis();
            }
            // if (Math.abs(desiredState.speedMetersPerSecond) > Constants.Swerve.MAX_SPEED
            // * 0.01) {
            double deltaTime = (System.currentTimeMillis() / 1000d) - (this.prevTime / 1000d);
            this.currentAcceleration = (desiredState.speedMetersPerSecond - this.prevSpeed) / deltaTime;
            this.prevTime = System.currentTimeMillis();
            this.prevSpeed = desiredState.speedMetersPerSecond;
            // }
            SmartDashboard.putNumber("Target Drive Mod " + this.moduleNumber,
            desiredState.speedMetersPerSecond);
            SmartDashboard.putNumber("Measured Mod Speed " + this.moduleNumber,
            this.getSpeed());
            // System.out.println("Accel: " + this.currentAcceleration);
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond, this.currentAcceleration));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.

        double angle = desiredState.angle.getDegrees() % 360.0;
        if (desiredState.speedMetersPerSecond != 0.0
                && (Math.abs(desiredState.speedMetersPerSecond) <= (swerveConfig.MAX_SPEED * 0.01))) {
            // System.out.println("IN Last angle");
            angle = lastAngle;
        }

        // System.out.println("Desired angle: " + angle);
        // SmartDashboard.putNumber("Target Mod Angle " + this.moduleNumber, angle);
        angleController.setReference(angle, ControlType.kPosition);
        lastAngle = angle;
        this.targetAngle = angle;
    }

    public void goToHome() {
        Rotation2d angle = getAngle();
        angleController.setReference(angle.getDegrees() - angle.getDegrees() % 360,
                ControlType.kPosition);
        lastAngle = angle.getDegrees() - angle.getDegrees() % 360;
    }

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/AbsAngle")
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(this.angleEncoder.getAbsolutePosition());
    }

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/Angle")
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.integratedAngleEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getSpeed(), this.getAngle());
    }

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/DriveSpeed")
    public double getSpeed() {
        return this.driveEncoder.getVelocity();
    }

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/DriveDistance")
    public double getDistance() {
        return this.driveEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.getDistance(), this.getAngle());
    }

    public SwerveModulePosition getRedPosition() {
        return new SwerveModulePosition(this.getDistance(), Rotation2d.fromDegrees(-this.getAngle().getDegrees()));
    }

    public CANSparkBase getDriveMotor() {
        return this.driveMotor;
    }
}
