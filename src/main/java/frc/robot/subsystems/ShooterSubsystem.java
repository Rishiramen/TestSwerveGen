package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX leftShooter, rightShooter, feeder;
    private bangbangCommand bang;
    private double targetRPM = 0.0;
    private CommandSwerveDrivetrain drivetrain;

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        leftShooter = new TalonFX(30);
        rightShooter = new TalonFX(31);
        feeder = new TalonFX(32);

        this.drivetrain = drivetrain;

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

        leftShooter.getConfigurator().apply(shooter);
        rightShooter.getConfigurator().apply(shooter);
        rightShooter.setControl(new Follower(30, MotorAlignmentValue.Opposed));

        feeder.getConfigurator().apply(tr);

        bang = new bangbangCommand(50);
        this.setDefaultCommand(bang);

        SmartDashboard.putNumber("targetRPM", targetRPM); // create it
    }

    public AngularVelocity getShooterVelocity()
    {
        return AngularVelocity.ofRelativeUnits(leftShooter.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter Current", leftShooter.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("left shooter rps", leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("left shooter velocity", this.getShooterVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("left shooter power", leftShooter.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("left shooter RPM", this.getShooterVelocity().in(RPM));

        targetRPM = SmartDashboard.getNumber("targetRPM", 0.0);

    }

    public Command reverseFeed()
    {
        return runOnce(() -> {
            this.openLoopFeeder(-1.0);
        });
    }

    public Command feed()
    {
        return new InstantCommand(() -> {
            this.openLoopFeeder(1.0);
        });
    }

    public Command stopFeed()
    {
        return new InstantCommand(() -> {
            this.openLoopFeeder(0.0);
        });
    }

    private void openLoopFeeder(double power)
    {
        feeder.set(power);
    }

    private void openLoopShooter(double power)
    {
        leftShooter.set(power);
    }

    public Command setTargetRPM(double RPM)
    {
        return runOnce(
            () -> {
                this.targetRPM = RPM;
            }
        );
    }

    final private class bangbangCommand extends Command
    {
        private ShooterSubsystem shooterSubsystem = ShooterSubsystem.this;
        BangBangController bangBangController;

        public bangbangCommand(double rpmTolerance)
        {
            this.bangBangController = new BangBangController();
            // this.bangBangController.setTolerance(rpmTolerance);

            addRequirements(shooterSubsystem);
        }

        @Override
        public void execute() {
            this.shooterSubsystem.openLoopShooter(
                bangBangController.calculate(
                    shooterSubsystem.getShooterVelocity().in(RPM),
                    shooterSubsystem.targetRPM
                )
            );
        }

        @Override
        public void end(boolean interrupted) {
            shooterSubsystem.openLoopShooter(0.0);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}