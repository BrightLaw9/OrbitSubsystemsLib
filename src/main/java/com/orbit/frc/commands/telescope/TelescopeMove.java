package com.orbit.frc.commands.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import com.orbit.frc.util.OrbitTimer;

public class TelescopeMove extends Command {
    /* The configuration class for this command */
    private TelescopeMoveConfig config;

    /* The end goal of this movement */
    private double targetPosition;

    /* Trapezoidal motion profile to be used */ 
    private TrapezoidProfile motionProfile;
    
    /* The start and end states of the motion, calculated on initialization */
    private TrapezoidProfile.State startState, endState;

    /* Timer to follow the motion profile */
    private OrbitTimer timer;

    /* Create a new TelescopeMove to the given angle using the given configuration 
     * @param config The configuration to be used for the command
     * @param position The target position for the command
     */
    public TelescopeMove(TelescopeMoveConfig config, double position) {
        this.config = config;
        this.targetPosition = position;
        this.timer = new OrbitTimer();

        addRequirements(config.telescope);
    }

    @Override
    public void initialize() {
        config.pid.reset();
        
        this.startState = new TrapezoidProfile.State(config.telescope.getPositionDegrees(), 0.0);
        this.endState = new TrapezoidProfile.State(this.targetPosition, 0.0);

        this.motionProfile = new TrapezoidProfile(config.motionProfileConstraints);

        this.timer.start();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State profileTarget = 
            motionProfile.calculate(
                this.timer.getTimeDeltaSec(),
            	this.startState, this.endState
            );
	
        double target = profileTarget.position;
        double input = config.telescope.getPositionMeters();
        double pidOutput = config.pid.calculate(target, input);
        config.telescope.setNormalizedVoltage(pidOutput);
    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
    }

    @Override
    public void end(boolean interrupted) {
        config.telescope.setVoltage(0.0); // stop
    }
}
