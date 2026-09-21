package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper;

public class SpinIntakeWheelMotor extends Command {

    private final Intake intakeSubsystem;
    private final Hopper hopperSubsystem;
    private final double speed;
    private CommandXboxController joystick = new CommandXboxController(0);

    public SpinIntakeWheelMotor(Intake intakeSubsystem, Hopper hopperSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.speed = speed;
        addRequirements(this.intakeSubsystem, this.hopperSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setWheelSpeedPercentage(speed);
        hopperSubsystem.setHopperSpeed(0.35);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopWheel();
        hopperSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}