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

public class HopperSubsystem extends SubsystemBase {
    private TalonFX hopper;

    public HopperSubsystem() {
        hopper = new TalonFX(35);

        TalonFXConfiguration config = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(30)
                        .withStatorCurrentLimit(30))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        hopper.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("hoppwe Current", hopper.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("hoppwe velo", hopper.getVelocity().getValueAsDouble());
    }

    public Command runForward() {
        return runOnce(() -> runRaw(0.5));
    }

    public Command runBackward() {
        return runOnce(() -> runRaw(-1));
    }

    public Command stop()
    {
        return runOnce(() -> {this.runRaw(0.0);});
    }

    public Command runTake(DoubleSupplier power) {
        return runOnce(() -> runRaw(power.getAsDouble()));
    }

    public void runRaw(double power) {
        hopper.set(power);
    }

    public Command runHopperOnce(double power) {
        return runOnce(() -> runRaw(power));
    }

}