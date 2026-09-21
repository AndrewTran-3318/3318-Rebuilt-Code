package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class HomeIntakeArm extends Command {
    private final Intake intakeSubsystem;
    private boolean isHomed;

    public HomeIntakeArm(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        SmartDashboard.putBoolean("isHomed", isHomed);

        // addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        isHomed = false;
    }

    @Override
    public void execute() {
        if(intakeSubsystem.getHallEffectSensorState())
        {
            intakeSubsystem.setArmSpeed(0);
            this.isHomed = true;
        }
        else
        {
            intakeSubsystem.setArmSpeed(Constants.INTAKE_LIFTER_HOMING_SPEED);
        }
          
    }

    @Override
    public boolean isFinished() {
        // TODOadd Timeout
        return isHomed;
    }

    @Override
    public void end(boolean interrupted) 
    {
        SmartDashboard.putBoolean("isHomed", isHomed);
        if (isHomed){
            intakeSubsystem.resetLifterEncoder();
        }
        intakeSubsystem.getPIDController()[0].setSetpoint(intakeSubsystem.getEncoder()[0].getPosition(), ControlType.kPosition);
        intakeSubsystem.getPIDController()[1].setSetpoint(intakeSubsystem.getEncoder()[1].getPosition(), ControlType.kPosition);
        isHomed = false;
    }

}