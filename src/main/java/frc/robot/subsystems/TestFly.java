package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestFly extends SubsystemBase {
    private TalonFX take, take2, transfer;
    private double originalCurrent;
    private double filteredCurrent;

    public TestFly() {
        take = new TalonFX(30);
        take2 = new TalonFX(31);
        transfer = new TalonFX(32);

        TalonFXConfiguration shooter = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(70)
                        .withStatorCurrentLimit(70))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));
        TalonFXConfiguration tr = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(25)
                        .withStatorCurrentLimit(25))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        take.getConfigurator().apply(shooter);
        take2.getConfigurator().apply(shooter);
        take2.setControl(new Follower(30, MotorAlignmentValue.Opposed));

        transfer.getConfigurator().apply(tr);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("shooter Current", take.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("shooter velo", take.getVelocity().getValueAsDouble());
    }

    public Command runTake(DoubleSupplier power) {
        return run(() -> runTakeRaw(power.getAsDouble()));
    }

    public Command runIntake(DoubleSupplier power) {
        return run(() -> runTakeRaw(power.getAsDouble()));
    }

    public Command runOuttake(DoubleSupplier power) {
        return run(() -> runTakeRaw(-power.getAsDouble()));
    }

    public void runTakeRaw(double power) {
        take.set(power);
        transfer.set(power);
    }

    public Command runTakeOnce(double power) {
        return runOnce(() -> runTakeRaw(power));
    }
}