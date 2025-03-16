// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
//import com.thethriftybot.ThriftyNova.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;




/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private static final CANcoder encoder_FL = new CANcoder(1);
  private static final CANcoder encoder_FR = new CANcoder(2);  
  private static final CANcoder encoder_BL = new CANcoder(3);
  private static final CANcoder encoder_BR = new CANcoder(4);

  private static final SparkMax Sparkmax_elevator = new SparkMax(14,MotorType.kBrushless);
  private static final SparkMax RightCoralIntake = new SparkMax(15, MotorType.kBrushless);
  private static final SparkMax LeftCoralIntake = new SparkMax(13, MotorType.kBrushless);

  private static final SparkMax PivotMax = new SparkMax(11, MotorType.kBrushless);
  private static RelativeEncoder RELATIVE_ENCODER = PivotMax.getEncoder();
  private double targetPosition = RELATIVE_ENCODER.getPosition(); 

  private static final SparkMax Coral_Intake = new SparkMax(10,MotorType.kBrushless);

  private static final double SCORE = 3.857;
  //commented
  private static final double LOAD = 10.2857;

  private final Timer autoTimer = new Timer();

  

  // private final SparkMax elevator = new SparkMax(4,MotorType.kBrushless);

  private static final XboxController XBOX_CONTROLLER = new XboxController(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  private Timer javaTimer;
private boolean isRunning = false;

@Override
public void autonomousInit() {
     var auto_command = m_robotContainer.getAutonomousCommand();
    if(auto_command!=null){
      auto_command.schedule();
    }
}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    // autoTimer.scheduleAtFixedRate(new TimerTask() {
    //   @Override
    //   public void run() {
    //     ChassisSpeeds backwardVelocity = new ChassisSpeeds(-0.5, 0, 0);
    //     m_robotContainer.drivebase.getSwerveDrive().driveFieldOriented(backwardVelocity);
          
    //   }
    // }, 1000);

    // m_robotContainer.drivebase.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(0,0,0));
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    SmartDashboard.putNumber("FL Angle: ", encoder_FL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("FR Angle: ", encoder_FR.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BL Angle: ", encoder_BL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BR Angle: ", encoder_BR.getAbsolutePosition().getValueAsDouble());

    

 
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //elevator.set(XBOX_CONTROLLER.getLeftX());

var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);

    PivotMax.configure(turnConfig, null, PersistMode.kNoPersistParameters);

    int PIVOT_RIGHT = XBOX_CONTROLLER.getLeftBumperButton() ? 1:0;
    int PIVOT_LEFT = XBOX_CONTROLLER.getRightBumperButton() ? 1:0;

    double pivot_speed = (PIVOT_RIGHT - PIVOT_LEFT)/10.0;

    PivotMax.set(pivot_speed);
  
    

    /*if (XBOX_CONTROLLER.getRightBumperButton() ) {
      double currentPosition = RELATIVE_ENCODER.getPosition();
      double error = LOAD - currentPosition;
      PivotMax.set(error*.1);
    } if (XBOX_CONTROLLER.getLeftBumperButton()) {
      double currentPosition = RELATIVE_ENCODER.getPosition();
      double error = - currentPosition;
      PivotMax.set(error*.1);
    }


    if (XBOX_CONTROLLER.getRightBumperButton()) {
      System.out.println(Coral_Intake.getAbsoluteEncoder().getPosition());
      coralIntakePID.calculate(Coral_Intake.getAbsoluteEncoder().getPosition(), 0.5);
    }
    */
    /* 
    System.out.println(PIVOT_LEFT);
    System.out.println(PIVOT_RIGHT);
    */
    double intake = XBOX_CONTROLLER.getAButton() ? 1:0;
    double outtake = XBOX_CONTROLLER.getYButton() ? 1:0;

    double algaeIntake = XBOX_CONTROLLER.getXButton() ? 1 : 0;
    double algaeOutake = XBOX_CONTROLLER.getBButton() ? 1 : 0;
    

    if (XBOX_CONTROLLER.getRightBumperButtonReleased() || XBOX_CONTROLLER.getLeftBumperButtonReleased())
    {
    targetPosition = RELATIVE_ENCODER.getPosition();
    System.out.println(targetPosition);
    }

    if (!XBOX_CONTROLLER.getRightBumperButton() && !XBOX_CONTROLLER.getLeftBumperButton()) {
      double currentPos = RELATIVE_ENCODER.getPosition();
      PivotMax.set((targetPosition - currentPos)*.05);
    }
    // System.out.println(pivot_speed);

    // if (XBOX_CONTROLLER.getRightBumperButtonReleased()) {
    //   PivotMax.set(-.05);
    // }

    Sparkmax_elevator.set((XBOX_CONTROLLER.getLeftTriggerAxis()-XBOX_CONTROLLER.getRightTriggerAxis())*2);
    Coral_Intake.set(intake-outtake);
    //LeftCoralIntake.set(algaeIntake-algaeOutake);
    RightCoralIntake.set(-(algaeIntake-algaeOutake));

    SmartDashboard.putNumber("FL Angle: ", encoder_FL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("FR Angle: ", encoder_FR.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BL Angle: ", encoder_BL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("BR Angle: ", encoder_BR.getAbsolutePosition().getValueAsDouble());

    
  }

  @Override
  public void testInit() {



    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
