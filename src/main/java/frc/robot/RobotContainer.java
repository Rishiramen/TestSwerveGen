// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Array;
import java.util.ArrayList;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private Translation3d goalPose3d = new Translation3d(0, 0, 0);

        public final CommandPS5Controller joystick = new CommandPS5Controller(0);
        private final ShooterSubsystem shooterSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ClimberSubsystem climberSubsystem;
        private final FeederSubsystem feederSubsystem;
        private final HopperSubsystem hopperSubsystem;
        private final CommandPS5Controller joystick2 = new CommandPS5Controller(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final SendableChooser<Command> autoChooser;
        private double speedMult =1;


        public RobotContainer() {
                DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
                shooterSubsystem = new ShooterSubsystem(drivetrain);
                intakeSubsystem = new IntakeSubsystem();
                feederSubsystem = new FeederSubsystem();
                climberSubsystem = new ClimberSubsystem();
                hopperSubsystem = new HopperSubsystem();

                feederSubsystem.setDefaultCommand(feederSubsystem.stop());

                intakeSubsystem.setDefaultCommand(
                                intakeSubsystem.runTake(() -> ((joystick.getL2Axis()+1)/2 - (joystick.getR2Axis()+1)/2 + (joystick2.getL2Axis()+1)/2-(joystick2.getR2Axis()+1)/2 ))
                        );
                hopperSubsystem.setDefaultCommand(
                                hopperSubsystem.runTake(() -> ((joystick.getL2Axis()+1)/2 - (joystick.getR2Axis()+1)/2 + (joystick2.getL2Axis()+1)/2-(joystick2.getR2Axis()+1)/2 )*.0 ));
                
                climberSubsystem.setDefaultCommand(

                                climberSubsystem.runTake(() -> joystick2.getLeftY()));
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed*speedMult)
                                                                .withDeadband(0.1 * MaxSpeed*speedMult) // Drive
                                                                // forward
                                                                // with
                                                                // negative Y
                                                                // (forward)
                                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed*speedMult)
                                                                .withDeadband(0.1 * MaxSpeed*speedMult) // Drive
                                                                                              // left
                                                                                              // with
                                                                                              // negative
                                                                                              // X
                                                                // (left)
                                                                .withRotationalRate(
                                                                                -joystick.getRightX() * MaxAngularRate*speedMult*.95) // Drive
                                                                                                                        // counterclockwise
                                                                                                                        // with
                                                                                                                        // negative
                                                                                                                        // X
                                                                                                                        // (left)
                                ));
                

                


                boolean useShooter = true;
                boolean useIntake = false;
                boolean useHopper = true;
                boolean useHang = true;


                NamedCommands.registerCommand("prime hang", useHang ? climberSubsystem.runTake(() -> -1).alongWith(new WaitCommand(1)).andThen(climberSubsystem.runTake(()->0)) : new InstantCommand());

                NamedCommands.registerCommand("start hopper", useHopper&&useShooter ? hopperSubsystem.runBackward().raceWith(new WaitCommand(4)) : new InstantCommand());
                
                NamedCommands.registerCommand("stop hopper", useHopper&&useShooter ? hopperSubsystem.stop().raceWith(new WaitCommand(.1)) : new InstantCommand());
                
                NamedCommands.registerCommand("start kicker", useHopper&&useShooter ? feederSubsystem.runBackward().raceWith(new WaitCommand(4)) : new InstantCommand());

                NamedCommands.registerCommand("stop kicker", useHopper&&useShooter ? feederSubsystem.stop().raceWith(new WaitCommand(.1)) : new InstantCommand());

                
                NamedCommands.registerCommand("deploy hang", useHang ? climberSubsystem.runTake(() -> -1).alongWith(new WaitCommand(2)).andThen(climberSubsystem.runTake(()->0)) : new InstantCommand());

                NamedCommands.registerCommand("stow intake", useIntake ? intakeSubsystem.setTargetOnly(IntakeSubsystem.State.STOWED) : new InstantCommand());
                NamedCommands.registerCommand("deploy intake", useIntake ? intakeSubsystem.setTargetOnly(IntakeSubsystem.State.DEPLOYED) : new InstantCommand());
                
                configureBindings();

                
                
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);


                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                // pdh.setSwitchableChannel(true);
                
                // hopperSubsystem.setDefaultCommand(
                //                 hopperSubsystem.runTake(() -> joystick.getRightY() + joystick2.getRightY() ));
                // intakeSubsystem.setDefaultCommand(
                //                 intakeSubsystem.runTake(() -> joystick.getRightY()+joystick2.getRightY() ));

                
                // new Trigger(() ->  Math.abs((joystick.getL2Axis()+1)/2 - (joystick.getR2Axis()+1)/2 + (joystick2.getL2Axis()+1)/2-(joystick2.getR2Axis()+1)/2) >.01 )
                //         .onTrue(
                //                 hopperSubsystem.runTake(() -> (joystick.getL2Axis()+1)/2 - (joystick.getR2Axis()+1)/2 + (joystick2.getL2Axis()+1)/2-(joystick2.getR2Axis()+1)/2 )
                //         );
                

                                // joystick.axisMagnitudeGreaterThan(PS5Controller.Axis.kL2.value, .01).onTrue(intakeSubsystem.runForward()).onFalse(intakeSubsystem.stop());
                // joystick.axisMagnitudeGreaterThan(PS5Controller.Axis.kR2.value, .01).onTrue(intakeSubsystem.runBackward().alongWith(hopperSubsystem.runBackward()))
                //         .onFalse(intakeSubsystem.stop().alongWith(hopperSubsystem.stop()));



                // joystick.cross().toggleOnTrue(hopper.runTake(() ->
                // -1).alongWith(fly.runTakeOnce(1)))
                // .toggleOnFalse(hopper.runTake(() -> 0).alongWith(fly.runTakeOnce(0)));
                

                joystick.cross()
                        .whileTrue(
                                 feederSubsystem.runBackward()
                                .alongWith(intakeSubsystem.runBackward())
                                .alongWith(hopperSubsystem.runBackward())
                        );

                new Trigger(() -> Math.abs((joystick.getL2Axis()+1)/2 - (joystick.getR2Axis()+1)/2)>.01 )
                        .whileTrue(new InstantCommand(() -> speedMult=.3)
                                .alongWith(feederSubsystem.runForward())
                                // .alongWith(new InstantCommand(() -> intakeSubsystem.updatePower(() -> (joystick.getL2Axis()+1)/2 - (joystick.getR2Axis()+1)/2)))
                                )
                        .whileFalse(new InstantCommand(() -> speedMult=1));
                
                
                
                // joystick.cross().whileTrue(feederSubsystem.runBackward());
                
                joystick.triangle().or(joystick2.triangle()).onTrue(intakeSubsystem.rezero());
                joystick2.circle().onTrue(intakeSubsystem.rezero(true));
                joystick.povUp().or(joystick2.povUp()).onTrue(intakeSubsystem.setTargetOnly(IntakeSubsystem.State.STOWED));
                joystick.povDown().or(joystick2.povDown()).onTrue(intakeSubsystem.setTargetOnly(IntakeSubsystem.State.DEPLOYED));

                joystick2.square().onTrue(new InstantCommand(shooterSubsystem::switchFixed));
                
                
                
                
                joystick.R1().whileTrue(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed*speedMult) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed*speedMult) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(
                                                                (drivetrain.getAutoAlignRotationalError().in(Degree))
                                                                                * .05 * MaxAngularRate) // Drive
                                                                                                        // counterclockwise
                                                                                                        // with
                                // negative X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
                joystick.PS().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(16.513 - 1.51,
                                (8.07 / 2) + ((8.07 / 2) - 3.721), new Rotation2d(Degree.of(180))))));
                // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
                // joystick.circle().whileTrue(drivetrain.applyRequest(
                // () -> point.withModuleDirection(
                // new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                
                joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                return autoChooser.getSelected();
        }
}
