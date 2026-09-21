package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class SpinHopperMotor extends Command {

    private final Hopper hopperSubsystem;
    private final double speed;

    public SpinHopperMotor(Hopper hopperSubsystem, double speed) {
        this.hopperSubsystem = hopperSubsystem;
        this.speed = speed;
        addRequirements(this.hopperSubsystem);
    }

    @Override
    public void execute() {
        hopperSubsystem.setHopperSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        hopperSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}