// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final ArmRotationSubsystem m_armRotationSubsystem = new ArmRotationSubsystem();
  private final ArmExtensionSubsystem m_armExtensionSubsystem = new ArmExtensionSubsystem();
  private final CANSparkMax m_appleMotor = new CANSparkMax(9, MotorType.kBrushless);
  private final DigitalInput m_atTopSwitch = new DigitalInput(5);
  private final DigitalInput m_atBottomSwitch = new DigitalInput(6);
  private final PneumaticHub m_pneumatics = new PneumaticHub();
  private final Timer m_delayTimer = new Timer();

  public enum FloatMovementStates {
    Unknown,
    StartLowering,
    WaitForDownSwitch,
    DelayAtDownSwitch,
    StartRaising,
    WaitForUpSwitch,
    DelayAtTopSwitch
  };

  private FloatMovementStates m_curretState = FloatMovementStates.Unknown;
  private boolean m_paradeMovement = false;
  private double m_appleSpeed = 0.5;
  private double m_pickAppleRotation = 150;
  private double m_pickAppleExtension = 25;
  private double m_deliverAppleRotation = 60;
  private double m_deliverAppleExtension = 0;
  private double m_afterSwitchesDelay = 2;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // turn off compressor
    m_pneumatics.disableCompressor();

    m_appleMotor.restoreFactoryDefaults();
    m_appleMotor.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("Apple Speed", 0.5);
    SmartDashboard.putNumber("Pick Apple Rotation", 150);
    SmartDashboard.putNumber("Pick Apple Extension", 25);
    SmartDashboard.putNumber("Deliver Apple Rotation", 60);
    SmartDashboard.putNumber("Deliver Apple Extension", 0);
    SmartDashboard.putNumber("After Switches Delay", 2);
    SmartDashboard.putBoolean("Run Float Program", false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_paradeMovement = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var runProgram = SmartDashboard.putBoolean("Run Float Program", false);
    // get start/stop from joystick buttons or dashboard
    if (runProgram == false)
    {
      if (m_paradeMovement)
      {
        m_paradeMovement = false;
        m_appleMotor.set(0);
        m_armRotationSubsystem.setArmDegrees(m_armRotationSubsystem.getArmAngle());
        m_armExtensionSubsystem.setArmExtensionPosition(m_armExtensionSubsystem.getArmExtensionPosition());
        m_curretState = FloatMovementStates.Unknown;
      }
    }
    else if (m_paradeMovement == false)
    {
      m_paradeMovement = true;
      m_curretState = FloatMovementStates.StartRaising;
    }

    if (m_paradeMovement)
    {
      NextMovemementState();
    }
  }

  private void NextMovemementState()
  {
    // BE CAREFUL - doing rotation and extension simultaneously right now!!!!
    switch (m_curretState)
    {
      case StartLowering:
        // start arm and extension movement
        m_armExtensionSubsystem.setArmExtensionPosition(m_deliverAppleExtension);
        m_armRotationSubsystem.setArmDegrees(m_deliverAppleRotation);
        // start apple movement
        m_appleMotor.set(-m_appleSpeed);
        m_curretState = FloatMovementStates.WaitForDownSwitch;
        break;
      case WaitForDownSwitch:
        if (m_atBottomSwitch.get())
        {
          m_appleMotor.set(0);
          m_curretState = FloatMovementStates.DelayAtDownSwitch;
          m_delayTimer.reset();
          m_delayTimer.start();
        }
        break;
      case DelayAtDownSwitch:
        if (m_delayTimer.hasElapsed(m_afterSwitchesDelay))
        {
          m_curretState = FloatMovementStates.StartRaising;
        }
        break;
      case StartRaising:
        m_armRotationSubsystem.setArmDegrees(m_pickAppleRotation);
        m_armExtensionSubsystem.setArmExtensionPosition(m_pickAppleExtension);
        m_appleMotor.set(m_appleSpeed);
        m_curretState = FloatMovementStates.WaitForUpSwitch;
        break;
      case WaitForUpSwitch:
        if (m_atTopSwitch.get())
        {
          m_appleMotor.set(0);
          m_curretState = FloatMovementStates.DelayAtTopSwitch;
          m_delayTimer.reset();
          m_delayTimer.start();
        }
        break;
      case DelayAtTopSwitch:
        if (m_delayTimer.hasElapsed(m_afterSwitchesDelay))
        {
          m_curretState = FloatMovementStates.StartLowering;
        }
        break;
      default:
        break;
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    m_appleSpeed = SmartDashboard.getNumber("Apple Speed", 0.5);
    m_pickAppleRotation = SmartDashboard.getNumber("Pick Apple Rotation", 150);
    m_pickAppleExtension = SmartDashboard.getNumber("Pick Apple Extension", 25);
    m_deliverAppleRotation = SmartDashboard.getNumber("Deliver Apple Rotation", 60);
    m_deliverAppleExtension = SmartDashboard.getNumber("Deliver Apple Extension", 0);
    m_afterSwitchesDelay = SmartDashboard.getNumber("After Switches Delay", 2);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
