// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }
    @Override
    public void robotInit(){
        // CameraServer.startAutomaticCapture();
        DataLogManager.start();
        // DriverStation.startDataLog(DataLogManager.getLog());
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        
        

        SmartDashboard.putNumber("Left y", m_robotContainer.joystick.getLeftY());
        SmartDashboard.putNumber("Left x", m_robotContainer.joystick.getLeftX());
        SmartDashboard.putNumber("Left trig", m_robotContainer.joystick.getL2Axis());
        SmartDashboard.putNumber("Right trig", m_robotContainer.joystick.getR2Axis());
        SmartDashboard.putNumber("right x", m_robotContainer.joystick.getRightX());
        SmartDashboard.putNumber("sum trig", Math.abs((m_robotContainer.joystick.getL2Axis()+1)/2 - (m_robotContainer.joystick.getR2Axis()+1)/2));
        SmartDashboard.putNumber("dist from goal",(m_robotContainer.drivetrain.getStateCopy().Pose.getTranslation().getDistance(CommandSwerveDrivetrain.goalPose2d)) );
        DogLog.log("roboPose",m_robotContainer.drivetrain.getStateCopy().Pose);
    

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
            m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);

    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        m_robotContainer.drivetrain.updateGoalPose();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_robotContainer.drivetrain.updateGoalPose();

    }

    @Override
    public void teleopPeriodic() {
         var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
        if (llMeasurement != null && llMeasurement.tagCount>0
                && Math.abs(m_robotContainer.drivetrain.getStateCopy().Speeds.omegaRadiansPerSecond) < 2.0) {
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
         SmartDashboard.putNumber("ll tag count", llMeasurement.tagCount);
        boolean[] boolArr = { llMeasurement != null, llMeasurement.tagCount > 0,
                Math.abs(m_robotContainer.drivetrain.getStateCopy().Speeds.omegaRadiansPerSecond) < 2.0 };

        SmartDashboard.putBooleanArray("ll mount", boolArr);
       
       
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
