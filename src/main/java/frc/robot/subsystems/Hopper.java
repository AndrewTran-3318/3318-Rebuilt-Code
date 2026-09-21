package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    //private final SparkMax hopperMotor;
    private final TalonFX hopperMotorKraken;

    public Hopper() {
        //hopperMotor = new SparkMax(Constants.HOPPER_ID, MotorType.kBrushless);
        hopperMotorKraken = new TalonFX(Constants.HOPPER_ID);

        TalonFXConfiguration hopperConfig = new TalonFXConfiguration(); //Motor configs

        hopperConfig.CurrentLimits.SupplyCurrentLimit = 30;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hopperConfig.CurrentLimits.StatorCurrentLimit = 80;
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        hopperConfig.Voltage.PeakForwardVoltage = 12.0;
        hopperConfig.Voltage.PeakReverseVoltage = -12.0;

        hopperConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

        hopperMotorKraken.getConfigurator().apply(hopperConfig);

        // SparkMaxConfig hopperConfig = new SparkMaxConfig();

        // hopperConfig
        //         .smartCurrentLimit(30)
        //         .idleMode(IdleMode.kBrake)
        //         .inverted(false);

        // hopperMotor.configure(
        //         hopperConfig,
        //         ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);

    }

    public void setHopperSpeed(double speed) {
        //hopperMotor.set(speed);
        hopperMotorKraken.set(speed);
    }

    public void stop() {
        //hopperMotor.stopMotor();
        hopperMotorKraken.stopMotor();
    }

    @Override
    public void periodic()
    {
        // SmartDashboard.putNumber("FeederVelo:", hopperMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("FeederTarget", hopperMotor.getClosedLoopController().getSetpoint());
        // SmartDashboard.putNumber("FeederPos:", hopperMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("FeederVelo:", hopperMotorKraken.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("FeederTarget", hopperMotorKraken.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("FeederPos:", hopperMotorKraken.getPosition().getValueAsDouble());
    }
}
