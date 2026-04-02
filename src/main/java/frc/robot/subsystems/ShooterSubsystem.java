package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.w3c.dom.css.CSSCharsetRule;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX leftShooter, rightShooter;
    // private bangbangCommand bang;
    private double targetRPM = 0.0;
    private CommandSwerveDrivetrain drivetrain;
    public boolean fixed = false;
    private final SysIdRoutine sysIdRoutine;
    private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0);

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        leftShooter = new TalonFX(30);
        rightShooter = new TalonFX(31);

        this.drivetrain = drivetrain;

        TalonFXConfiguration shooter = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(50)
                        .withStatorCurrentLimit(50))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));
        shooter.Slot0.kP = 0.50; 
        shooter.Slot0.kI = 0.0;
        shooter.Slot0.kD = 0.0;
        shooter.Slot0.kS = 0.25;
        shooter.Slot0.kG = 0.0;
        shooter.Slot0.kV = 1.0 / 8.35; // 1.0 V per 8.35 RPS; might run a little high at high RPS
        shooter.Slot0.kA = 0.0;


        leftShooter.getConfigurator().apply(shooter);
        rightShooter.getConfigurator().apply(shooter);
        rightShooter.setControl(new Follower(30, MotorAlignmentValue.Opposed));

        // bang = new bangbangCommand(50);
        this.setDefaultCommand(run(() -> setVelocity(targetRPM)));
        // this.setDefaultCommand(run(() -> openLoopShooter(2)));

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        leftShooter.setVoltage(volts.in(Volts));
                    },
                    log -> {
                        log.motor("shooter1")
                                .voltage(leftShooter.getMotorVoltage().getValue())
                                .angularVelocity(leftShooter.getVelocity().getValue())
                                .angularPosition(leftShooter.getPosition().getValue());
                    },
                    this));
        SmartDashboard.putNumber("targetRPM", targetRPM); // create it
    }

    public AngularVelocity getShooterVelocity() {
        return AngularVelocity.ofRelativeUnits(leftShooter.getVelocity().getValueAsDouble(), RotationsPerSecond);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter Current", leftShooter.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("left shooter rps", leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("left shooter velocity", this.getShooterVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("left shooter power", leftShooter.get());
        SmartDashboard.putNumber("left shooter RPM", this.getShooterVelocity().in(RPM));
        SmartDashboard.putBoolean("fixed", fixed);

        targetRPM = (!fixed) ? Constants.getRPM(drivetrain.getDistFromGoal().in(Meter)) : 3200;
        // targetRPM = SmartDashboard.getNumber("targetRPM",(!fixed) ? Constants.getRPM(drivetrain.getDistFromGoal().in(Meter)) : 3200);
        SmartDashboard.putNumber("targetRPM", targetRPM);
        SmartDashboard.putString("shooter Command", getCurrentCommand() == null ? "null" : getCurrentCommand().getName());

    }

    public void switchFixed() {
        fixed = !fixed;
    }
    

    private void openLoopShooter(double power) {
        leftShooter.set(power);
        // leftShooter.setControl(new DutyCycleOut(power));
    }

    public void setVelocity(double rpm){
        leftShooter.setControl(flywheelVelocity.withVelocity(rpm / 60.0).withSlot(0));
    }

    public Command setTargetRPM(double RPM) {
        return runOnce(
                () -> {
                    this.targetRPM = RPM;
                });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return sysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return sysIdRoutine.dynamic(dir);
    }

    // final private class bangbangCommand extends Command
    // {
    //     private ShooterSubsystem shooterSubsystem = ShooterSubsystem.this;
    //     BangBangController bangBangController;

    //     public bangbangCommand(double rpmTolerance)
    //     {
    //         this.bangBangController = new BangBangController();
    //         // this.bangBangController.setTolerance(rpmTolerance);

    //         addRequirements(shooterSubsystem);
    //     }

    //     @Override
    //     public void execute() {
    //         this.shooterSubsystem.openLoopShooter(
    //             bangBangController.calculate(
    //                 shooterSubsystem.getShooterVelocity().in(RPM),
    //                 shooterSubsystem.targetRPM
    //             )//+(1/3400.0)*shooterSubsystem.targetRPM
    //         );
    //     }

    //     @Override
    //     public void end(boolean interrupted) {
    //         shooterSubsystem.openLoopShooter(0.0);
    //     }

    //     @Override
    //     public boolean isFinished() {
    //         return false;
    //     }
    // }
}