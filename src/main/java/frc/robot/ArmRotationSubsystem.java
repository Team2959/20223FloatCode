// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotationSubsystem extends SubsystemBase {
  public static final double kArmHomePosition = 60;
  private static final double kArmRotatorP = 0.02;
  private static final double kArmRotatorI = 0.00002;
  private static final double kArmRotatorD = 0;
  private static final double kArmRotatorMaxVelocity = 500; // degrees per second
  private static final double kArmRotatorMaxAccel = 500; // degrees per second per second
  private double m_lastArmRotationTarget = kArmHomePosition;

  private CANSparkMax m_armRotatorMotor = new CANSparkMax(7, MotorType.kBrushless);
  private DigitalInput m_armRotatorEncoderDigitalInput = new DigitalInput(8);
  private DutyCycle m_armRotatorEncoderDutyCycle = new DutyCycle(m_armRotatorEncoderDigitalInput);

  private ProfiledPIDController m_armRotatorMotorProfiledPidController = new ProfiledPIDController(kArmRotatorP,
      kArmRotatorI, kArmRotatorD,
      new Constraints(kArmRotatorMaxVelocity, kArmRotatorMaxAccel));

  private SparkMaxRelativeEncoder m_armRotatorEncoder;

  public ArmRotationSubsystem() {
    m_armRotatorMotor.setIdleMode(IdleMode.kBrake);
    // m_armRotatorMotor.setIdleMode(IdleMode.kCoast);
    m_armRotatorEncoder = (SparkMaxRelativeEncoder) m_armRotatorMotor.getEncoder();

    setArmDegrees(kArmHomePosition);
  }

  public double lastArmRotationTarget() {
    return m_lastArmRotationTarget;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + "/Arm Rotation Encoder Position", getArmAngle());

    double rawProfiled = m_armRotatorMotorProfiledPidController.calculate(getArmAngle());
    m_armRotatorMotor.set(rawProfiled);
  }

  public void setArmDegrees(double degrees) {
    degrees = Math.min(220, degrees);
    degrees = Math.max(-75, degrees);
    // profiled PID control
    m_armRotatorMotorProfiledPidController.setGoal(degrees);
    m_lastArmRotationTarget = degrees;
  }

  public void stopArmRotatorMotor() {
    setArmDegrees(getArmAngle());
  }

  public boolean isArmRotatorAtSetpoint() {
    return Math.abs(m_lastArmRotationTarget - getArmAngle()) < 5;
  }

  public double getArmAngle() {
    var angleInDegress = m_armRotatorEncoderDutyCycle.getOutput() * 360;
    if (angleInDegress > 240)
      angleInDegress -= 360;
    return angleInDegress;
  }

  public void smartDashboardInit() {
    SmartDashboard.putBoolean(getName() + "/Arm Rot Prof Const Enable", false);
    SmartDashboard.putNumber(getName() + "/Arm Rotator Max Accel", kArmRotatorMaxAccel);
    SmartDashboard.putNumber(getName() + "/Arm Rotator Max Vel", kArmRotatorMaxVelocity);

    SmartDashboard.putNumber(getName() + "/Arm Rotator P", m_armRotatorMotorProfiledPidController.getP());
    SmartDashboard.putNumber(getName() + "/Arm Rotator I", m_armRotatorMotorProfiledPidController.getI());
    SmartDashboard.putNumber(getName() + "/Arm Rotator D", m_armRotatorMotorProfiledPidController.getD());
  }

  public void smartDashboardUpdate() {
    if (SmartDashboard.getBoolean(getName() + "/Arm Rot Prof Const Enable", false)) {
      var maxAccelRot = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Accel", kArmRotatorMaxAccel);
      var maxVelRot = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Vel", kArmRotatorMaxVelocity);
      m_armRotatorMotorProfiledPidController.setConstraints(new Constraints(maxVelRot, maxAccelRot));
    }

    var rotP = SmartDashboard.getNumber(getName() + "/Arm Rotator P", kArmRotatorP);
    var rotI = SmartDashboard.getNumber(getName() + "/Arm Rotator I", kArmRotatorI);
    var rotD = SmartDashboard.getNumber(getName() + "/Arm Rotator D", kArmRotatorD);
    SmartDashboard.putNumber(getName() + "/Arm Rotator Velocity", m_armRotatorEncoder.getVelocity());

    m_armRotatorMotorProfiledPidController.setPID(rotP, rotI, rotD);
  }
}
