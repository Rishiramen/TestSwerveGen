package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX take;
    private double originalCurrent;
    private double filteredCurrent;

    private State state = State.STOWED;

    private Angle START_HORIZONTAL_OFFSET = Degree.of(0);
    public static final double WRIST_RATIO = 1.0/125.0 * (10.0/32.0);
    public static double testPos= 15;


    public enum State {
        TEST(-1),
        DEPLOYED(120),
        TRANSFER(45),
        STOWED(0);

        private final Angle angle;

        State(Angle angle) {
            this.angle = angle;
        }
        State(double degrees) {
            this(Units.Degrees.of(degrees));
        }

        public Angle angle() {
            if(angle.in(Degree) == -1) return Units.Degrees.of(testPos);
            return angle;
        }
    }

    public State getState() {
        return state;
    }
    public Angle getTarget() {
        return state.angle();
    }
    public Command runTo(){
        PositionVoltage m_request = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
        return run(() -> take.setControl(m_request.withPosition(getTarget().div(WRIST_RATIO))));
    }

    public Command setTargetOnly(State state) {        
        return runOnce(() -> this.state = state).andThen(runTo());        
    }

    public ClimberSubsystem() {
        take = new TalonFX(34);

        TalonFXConfiguration shooter = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withStatorCurrentLimit(40))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));
        shooter.Slot0.kP = 0.5; // start small, tune up
        shooter.Slot0.kI = 0.0;
        shooter.Slot0.kD = 0.0;

        take.getConfigurator().apply(shooter);
        take.setPosition(START_HORIZONTAL_OFFSET.div(WRIST_RATIO));


    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("shooter Current", take.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("shooter velo", take.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("hang pos", take.getPosition().getValue().times(1.0 / 125.0).in(Degrees));
    }

    public Command runTake(DoubleSupplier power) {
        return runOnce(() -> runTakeRaw(power.getAsDouble()));
    }

    public Command runIntake(DoubleSupplier power) {
        return run(() -> runTakeRaw(power.getAsDouble()));
    }

    public Command runOuttake(DoubleSupplier power) {
        return run(() -> runTakeRaw(-power.getAsDouble()));
    }

    public void runTakeRaw(double power) {
        // take.set(power);
        take.setControl(new DutyCycleOut(power).withEnableFOC(true));
    }

    public Command runTakeOnce(double power) {
        return runOnce(() -> runTakeRaw(power));
    }
}