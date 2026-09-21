package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    private final SparkMax feederMotor;

    public Feeder() {
        feederMotor = new SparkMax(Constants.FEEDER_SHOOTER_NEO_ID, MotorType.kBrushless);
        SparkMaxConfig feederConfig = new SparkMaxConfig();

        feederConfig
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kCoast)
                .inverted(false);

        feederMotor.configure(
                feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    public void stop() {
        feederMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("FeederVelo:", feederMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("FeederTarget", feederMotor.getClosedLoopController().getSetpoint());
        SmartDashboard.putNumber("FeederPos:", feederMotor.getEncoder().getPosition());
    }
}
