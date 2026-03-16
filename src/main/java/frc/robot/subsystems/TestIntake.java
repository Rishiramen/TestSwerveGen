package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestIntake extends SubsystemBase {
    private TalonFX take, take2, pivot;

    public TestIntake() {
        take = new TalonFX(33);
        take2 = new TalonFX(38);
        pivot = new TalonFX(37);

        TalonFXConfiguration config = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(30)
                        .withStatorCurrentLimit(30))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        take.getConfigurator().apply(config);
        take2.getConfigurator().apply(config);
        take2.setControl(new Follower(33, MotorAlignmentValue.Opposed));
        pivot.getConfigurator().apply(config);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("hoppwe Current", take.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake pivot pos", pivot.getPosition().getValue().times(1.0 / 25.0).in(Degree));

        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("hoppwe velo", take.getVelocity().getValueAsDouble());
    }

    public Command runForward() {
        return run(() -> runRaw(0.5));
    }

    public Command runBackward() {
        return run(() -> runRaw(-0.5));
    }

    public Command runTake(DoubleSupplier power) {
        return run(() -> runRaw(power.getAsDouble()));
    }

    public Command runPivot(DoubleSupplier power) {
        return run(() -> runPivotRaw(power.getAsDouble()));
    }

    public void runRaw(double power) {
        take.set(power);
    }

    public void runPivotRaw(double power) {
        pivot.set(power);
    }

    public Command runIntakeOnce(double power) {
        return runOnce(() -> runRaw(power));
    }

}