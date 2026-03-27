package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX take, take2, pivot;
    private double power =0;

    private State state = State.STOWED;

    private Angle START_HORIZONTAL_OFFSET = Degree.of(85);
    public static final double WRIST_RATIO = 1.0/75.0;


    public enum State {
        DEPLOYED(15),
        STOWED(95);

        private final Angle angle;

        State(Angle angle) {
            this.angle = angle;
        }
        State(double degrees) {
            this(Units.Degrees.of(degrees));
        }

        public Angle angle() {
            return angle;
        }
    }

    public State getState() {
        return state;
    }
    public Angle getTarget() {
        return state.angle;
    }

    public Angle getPosition() {
        return pivot.getPosition().getValue().times(WRIST_RATIO).plus(START_HORIZONTAL_OFFSET);
    }
    public AngularVelocity getVelocity() {
        return pivot.getVelocity().getValue().times(WRIST_RATIO);
    }
    public AngularAcceleration getAcceleration() {
        return pivot.getAcceleration().getValue().times(WRIST_RATIO);
    }

    public Current getCurrent() {
        return pivot.getStatorCurrent().getValue();
    }

    // public Command setTargetStay(State state) {
    //     return runOnce(() -> this.state = state).andThen(runTo()).andThen(stay());        
    // }
    public Command setTargetOnly(State state) {        
        return runOnce(() -> this.state = state).andThen(runTo());        
    }

    public Angle getError() {
        return getState().angle.minus(getPosition());
    }
    public Command rezero(){
        return runOnce(() -> pivot.setPosition(START_HORIZONTAL_OFFSET.div(WRIST_RATIO)));
    }
    public Command rezero(boolean down){
        return runOnce(() -> pivot.setPosition(State.DEPLOYED.angle.div(WRIST_RATIO)));
    }


    public Command runTo(){
        PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        return run(() -> pivot.setControl(m_request.withPosition(getTarget().div(WRIST_RATIO))));
    }






    

    public IntakeSubsystem() {
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
        TalonFXConfiguration configArm = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(30)
                        .withStatorCurrentLimit(30))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        configArm.Slot0.kP = 0.3; // start small, tune up
        configArm.Slot0.kI = 0.0;
        configArm.Slot0.kD = 0.0;

        take.getConfigurator().apply(config);
        take2.getConfigurator().apply(config);
        take2.setControl(new Follower(33, MotorAlignmentValue.Opposed));
        pivot.getConfigurator().apply(configArm);

        pivot.setPosition(START_HORIZONTAL_OFFSET.div(WRIST_RATIO));

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("intake roller Current", take.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("intake pivot pos", pivot.getPosition().getValue().times(1.0 / 25.0).in(Degree));

        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("Pivot Current (A)", getCurrent().in(Units.Amp));
        SmartDashboard.putNumber("Pivot Power", pivot.get());
        SmartDashboard.putNumber("Pivot Position", getPosition().in(Units.Degree));
        SmartDashboard.putNumber("Pivot PID Position", getPosition().in(Degree));
        SmartDashboard.putNumber("Pivot PID Position Target", getTarget().div(WRIST_RATIO).in(Degree));


        SmartDashboard.putNumber("Pivot Velo", getVelocity().in(Units.DegreesPerSecond));
        SmartDashboard.putNumber("Pivot Accel", getAcceleration().in(Units.DegreesPerSecondPerSecond));


    }

    public Command runForward() {
        return runOnce(() -> {updatePower(()->.5);});
    }

    public Command runBackward() {
        return runOnce(() -> {updatePower(()->-.5);});
    }

    public Command stop() {
        return runOnce(() -> {updatePower(()->-.5);});
    }

    public void updatePower(DoubleSupplier power){
        this.power = power.getAsDouble();
    }

    public Command runTake() {
        return run(() -> runRaw(power));
    }


    public void runRaw(double power) {
        take.set(power);
    }

   
    

}