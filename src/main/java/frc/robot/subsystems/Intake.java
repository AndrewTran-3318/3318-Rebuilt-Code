package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax intakeArmMotorRight;
    private final SparkMax intakeArmMotorLeft;
    private final TalonFX intakeWheelMotorLeft;
    private final TalonFX intakeWheelMotorRight;
    private final DutyCycleOut percentageSpeed = new DutyCycleOut(0);

    private final RelativeEncoder encoderRight;
    private final RelativeEncoder encoderLeft;
    private RelativeEncoder[] encoders = new RelativeEncoder[2];
    private SparkClosedLoopController pidControllerRight;
    private SparkClosedLoopController pidControllerLeft;
    private SparkClosedLoopController[] pidControllers = new SparkClosedLoopController[2];
    private static DigitalInput hallEffectHomingSensor = new DigitalInput(Constants.HALL_EFFECT_HOMING_DIO_PORT);

    //HALL_EFFECT_DEBOUNCE_SEC might need tuning
    private final Debouncer hallEffectDebouncer = new Debouncer(Constants.HALL_EFFECT_DEBOUNCE_SEC, Debouncer.DebounceType.kBoth);
    //private SparkMaxConfig cSparkMaxConfig = new SparkMaxConfig();


    public Intake() {
        intakeArmMotorRight = new SparkMax(Constants.INTAKE_ARM_RIGHT_ID, MotorType.kBrushless); // neo
        intakeArmMotorLeft = new SparkMax(Constants.INTAKE_ARM_LEFT_ID, MotorType.kBrushless);
        
        encoderRight = intakeArmMotorRight.getEncoder();
        encoderLeft = intakeArmMotorLeft.getEncoder();
        encoders[0] = encoderRight;
        encoders[1]= encoderLeft;
        pidControllerRight = intakeArmMotorRight.getClosedLoopController();
        pidControllerLeft = intakeArmMotorLeft.getClosedLoopController();
        pidControllers[0]=pidControllerRight;
        pidControllers[1] = pidControllerLeft;


        //these need tuning
        //cSparkMaxConfig.closedLoop.pid(0.2,0,0);
        //intakeArmMotor.configure(cSparkMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig ArmConfig = new SparkMaxConfig(); //Motor Configs

        ArmConfig
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        
        ArmConfig.closedLoop.pid(0.02,0,0);

        intakeArmMotorRight.configure(
                ArmConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        
 //Distinction, sets left intake arm motor as a follower of the right arm

        intakeArmMotorLeft.configure(
                ArmConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        

        intakeWheelMotorLeft = new TalonFX(Constants.INTAKE_WHEEL_KRAKEN_ID_LEFT); // kraken
        intakeWheelMotorRight = new TalonFX(Constants.INTAKE_WHEEL_KRAKEN_ID_RIGHT);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration(); //Motor Configs
        
        intakeConfig.Slot0.kP = 0.12;
        intakeConfig.Slot0.kI = 0.0;
        intakeConfig.Slot0.kD = 0.0;

        intakeConfig.Slot0.kV = 0.11;
        intakeConfig.Slot0.kS = 0.2;
        intakeConfig.Slot0.kA = 0.0;

        intakeConfig.CurrentLimits.SupplyCurrentLimit = 30;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeConfig.Voltage.PeakForwardVoltage = 12.0;
        intakeConfig.Voltage.PeakReverseVoltage = -12.0;

        intakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

        intakeWheelMotorRight.getConfigurator().apply(intakeConfig);
        intakeWheelMotorLeft.getConfigurator().apply(intakeConfig);
        
        //setting rightIntakeWheelMotor as a follower of LeftIntakeWheelMotor
        intakeWheelMotorRight.setControl(new Follower(Constants.INTAKE_WHEEL_KRAKEN_ID_LEFT, MotorAlignmentValue.Opposed));
    }

    // neo methods (arm)
    public void setArmSpeed(double speed) {
        intakeArmMotorRight.set(speed);
        intakeArmMotorLeft.set(-speed/2);
    }

    public void stop() {
        intakeArmMotorRight.stopMotor();
        intakeArmMotorLeft.stopMotor();
    }

    public boolean isArmUp(){
        /* 
            double pos = encoder.getPosition();
            return pos >= Constants.INTAKE_LIFTER_UP_POS - 0.05 && pos <= Constants.INTAKE_LIFTER_UP_POS + 0.05;
        */
        return getHallEffectSensorState();
    }

    public RelativeEncoder[] getEncoder(){
        return encoders;
    }

    public SparkClosedLoopController[] getPIDController(){
        return pidControllers;
    }

    // kraken methods (flywheel)
    public void setWheelSpeedPercentage(double speed) {
        intakeWheelMotorLeft.setControl(percentageSpeed.withOutput(speed));
    }

    public void stopWheel() {
        intakeWheelMotorLeft.setControl(percentageSpeed.withOutput(0));
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("IntakeLifterRightPos:", intakeArmMotorRight.getEncoder().getPosition());
        SmartDashboard.putNumber("IntakeWheelLeftVelocity:", intakeWheelMotorLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("IntakeWheelRightVelocity:", intakeWheelMotorRight.getVelocity().getValueAsDouble());
    }

    public boolean getHallEffectSensorState()
    {
        //Added debouncer to get rid of noise
        return hallEffectDebouncer.calculate(!hallEffectHomingSensor.get());
    }

    public void resetLifterEncoder()
    {
        intakeArmMotorRight.getEncoder().setPosition(0);
        intakeArmMotorLeft.getEncoder().setPosition(0);
    }
}

// kraken for rollers and neo for up and down