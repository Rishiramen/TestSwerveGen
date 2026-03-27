package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private TalonFX feeder;

    public FeederSubsystem() {
        feeder = new TalonFX(32);

       TalonFXConfiguration tr = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(25)
                        .withStatorCurrentLimit(25))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        feeder.getConfigurator().apply(tr);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("feeder Current", feeder.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("feeder velo", feeder.getVelocity().getValueAsDouble());
    }

    public Command runForward() {
        return run(() -> runRaw(-0.5));
    }

    public Command runBackward() {
        return run(() -> runRaw(1));
    }

    public Command stop()
    {
        return run(() -> runRaw(0.0));
    }
    
    public Command runForwardOnce() {
        return runOnce(() -> runRaw(-0.5));
    }

    public Command runBackwardOnce() {
        return runOnce(() -> runRaw(1));
    }

    public Command stopOnce()
    {
        return runOnce(() -> runRaw(0.0));
    }



    public Command runTake(DoubleSupplier power) {
        return runOnce(() -> runRaw(power.getAsDouble()));
    }

    public void runRaw(double power) {
        feeder.set(power);
    }

    public Command runHopperOnce(double power) {
        return runOnce(() -> runRaw(power));
    }

}