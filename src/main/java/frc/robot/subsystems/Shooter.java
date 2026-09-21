package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonFX rightShooterMotor = new TalonFX(Constants.RIGHT_SHOOTER_KRAKEN_ID);
    private final TalonFX leftShooterMotor = new TalonFX(Constants.LEFT_SHOOTER_KRAKEN_ID);
    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

    public Shooter() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        
        shooterConfig.Slot0.kP = 0.12;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;

        shooterConfig.Slot0.kV = 0.11;
        shooterConfig.Slot0.kS = 0.2;
        shooterConfig.Slot0.kA = 0.0;

        shooterConfig.CurrentLimits.SupplyCurrentLimit = 30;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConfig.CurrentLimits.StatorCurrentLimit = 80;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shooterConfig.Voltage.PeakForwardVoltage = 12.0;
        shooterConfig.Voltage.PeakReverseVoltage = -12.0;

        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

        rightShooterMotor.getConfigurator().apply(shooterConfig);
        leftShooterMotor.getConfigurator().apply(shooterConfig);

        //setting rightShooterMotor as a follower of LeftShooterMotor
        rightShooterMotor.setControl(new Follower(Constants.LEFT_SHOOTER_KRAKEN_ID, MotorAlignmentValue.Opposed));
    }

    public void setShooterSpeed(double speed) { //SPEED IN RPS
        leftShooterMotor.setControl(shooterVelocityRequest.withVelocity(speed));
    }

    public void stop() {
        leftShooterMotor.setControl(shooterVelocityRequest.withVelocity(0));
    }

    /**
     * Calculates the flywheel RPM with no slip/backspin and perfect transfer to
     * reach target
     * 
     * @see https://www.desmos.com/calculator/x4skexyoim for plotter, derivation
     *      from 2d kinematics
     * @param deltaDistance Horizontal Distance from shooter exit to hub aim point
     * 
     */
    public double getTheoreticalRPS(double deltaDistance) {
        final double distanceOffset = 0.0; // Increase to aim further than middle of hub
        final double hoodAngleRadians = deg2rad(Constants.HOODANGLEDEGREES); // from horizontal
        final double deltaHeight = inch2met(Constants.HUB_HEIGHT) - inch2met(Constants.SHOOTERHEIGHTINCHES);

        double shootingDistance = deltaDistance + distanceOffset;

        double velocity = (shootingDistance) / Math.cos(hoodAngleRadians) *
                Math.sqrt(
                        (-0.5 * Constants.g) / (deltaHeight - shootingDistance * Math.tan(hoodAngleRadians)));

        double RPSTheoretical = velocity / (Constants.FLY_WHEEL_RADIUS * 2 * Math.PI);

        return RPSTheoretical;
    }

    /**
     * @See 
     * @param deltaDistance Horizontal Distance from shooter exit to hub aim point
     * @return The necessary RPS based on physics and empirical data to make the shot
     */
    public double getCorrectedRPS(double deltaDistance) {
        final double distanceOffset = 0.0; // Increase to aim further than middle of hub
        final double hoodAngleRadians = deg2rad(Constants.HOODANGLEDEGREES); // from horizontal
        final double deltaHeight = inch2met(Constants.HUB_HEIGHT) - inch2met(Constants.SHOOTERHEIGHTINCHES);

        double shootingDistance = deltaDistance + distanceOffset;

        double velocity = Constants.K1Offset * (shootingDistance) / Math.cos(hoodAngleRadians) *
                Math.sqrt(
                        (-0.5 * Constants.g) / (deltaHeight - Constants.K2Offset * shootingDistance * Math.tan(hoodAngleRadians)));

        double RPS = velocity / (Constants.FLY_WHEEL_RADIUS * 2 * Math.PI);

        return RPS;
    }

    /**
     * Converts degrees to radians
     * 
     * @param degrees Input in degrees
     * @return Output in radians
     */
    private double deg2rad(double degrees) // degrees to radians
    {
        return degrees * Math.PI / 180;
    }

    /**
     * Converts inches to meters (1 Inch -> 0.0254 meters)
     * 
     * @param inches Input in inches
     * @return Output in Meters
     */
    public double inch2met(double inches) {
        return inches * 2.54 / 100.0;
    }

    //Congerts RPM to RPS
    public double RPM2RPS(double RPM) {
        return RPM / 60.0;
    }

    public boolean isAtSpeed(double targetRPS) {
        double leftRPS  = Math.abs(getLeftRPS());
        double rightRPS = Math.abs(getRightRPS());
        return Math.abs(leftRPS  - targetRPS) <= 1.5
            && Math.abs(rightRPS - targetRPS) <= 1.5;
    }

    public double getLeftRPS() {
        return leftShooterMotor.getVelocity().getValueAsDouble();
    }

    public double getRightRPS() {
        return rightShooterMotor.getVelocity().getValueAsDouble();
        
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("LeftShooterSpeed", getLeftRPS());
        SmartDashboard.putNumber("RightShooterSpeed", getRightRPS());
        SmartDashboard.putNumber("LeftShooterTarget", leftShooterMotor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("RightShooterTarget", rightShooterMotor.getClosedLoopReference().getValueAsDouble());
    }

    public Pose2d findHubCoords(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d hub = null;
        if (alliance.isPresent()) {
            hub = (alliance.get() == Alliance.Red) ? Constants.RED_HUB_CORDS : Constants.BLUE_HUB_CORDS;
        }
        return hub;
    }
}




// 1 kraken to shoot 1 neo to index for each shooter (2 shooters)