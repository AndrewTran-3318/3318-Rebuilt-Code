package frc.robot.Commands;

import frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj2.command.Command;

public class MoveIntakeArm extends Command {
    private final Intake intakeSubsystem;
    private double speed;

    public MoveIntakeArm(Intake intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //start moving to the up or down position based on if it's currently up or down
    }

    @Override
    public void execute() {
        intakeSubsystem.setArmSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        //continuously check if it has reached the up/down position (fault tolerance of 0.05)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}