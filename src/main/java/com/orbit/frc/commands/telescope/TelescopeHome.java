package com.orbit.frc.commands.telescope;

import edu.wpi.first.wpilibj2.command.Command;

public class TelescopeHome extends Command {
    /* The configuration class for this command */
    private TelescopeHomeConfig config;

    /* Create a new TelescopeMove to the given angle using the given configuration 
     * @param config The configuration to be used for the command 
     * */
    public TelescopeHome(TelescopeHomeConfig config) {
        this.config = config;
    }

    @Override
    public void initialize() {
        return; 
    }

    @Override
    public void execute() {
        this.config.telescope.setNormalizedVoltage(this.config.speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.config.telescope.setNormalizedVoltage(0.0);
        this.config.telescope.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return this.config.telescope.getLimitSwitch();
    }
}
