package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class SpinFeederMotor extends Command {

    private final Feeder feederSubsystem;
    private final double speed;

    public SpinFeederMotor(Feeder feederSubsystem, double speed) {
        this.feederSubsystem = feederSubsystem;
        this.speed = speed;
        addRequirements(this.feederSubsystem);
    }

    @Override
    public void execute() {
        feederSubsystem.setFeederSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}