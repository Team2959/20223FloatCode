// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {
    private static final int kSmartMaxVel = 10000;
    private static final int kSmartMaxAccel = 10000;
    // private static final double kArmExtensionP = 0.02;  // orig kP from St Joe: 0.02
    // private static final double kArmExtensionI = 0;  // orig KI from St Joe: 0
    private static final double kArmExtensionP = 0.15; // POSTION: 0.15 //0.00035; //CF 0.00035 ALUM: 0.000075
    private static final double kArmExtensionI = 0; //0.000001;
    private static final double kArmExtensionD = 0;
    private static final double kArmExtensionFF = 0;
    private static final double kArmExtensionIzone = 0;
    // private static final double kSmartAllowedError = 0;

    private CANSparkMax m_armExtensionMotor = new CANSparkMax(8, MotorType.kBrushless);
    private SparkMaxRelativeEncoder m_extensionEncoder;
    private SparkMaxPIDController m_armExtensionMotorPidController;
    private double m_lastArmExtensionTarget = 0; // Arm by default starts fully retracted, unless onAutoInit() is called

    /** Creates a new ArmExtensionSubsystem. */
    public ArmExtensionSubsystem() {
        m_armExtensionMotor.restoreFactoryDefaults();
        m_armExtensionMotor.setIdleMode(IdleMode.kBrake);
        // m_armExtensionMotor.setIdleMode(IdleMode.kCoast);

        m_extensionEncoder = (SparkMaxRelativeEncoder) m_armExtensionMotor.getEncoder();
        m_armExtensionMotorPidController = m_armExtensionMotor.getPIDController();

        m_armExtensionMotorPidController.setP(kArmExtensionP);
        m_armExtensionMotorPidController.setI(kArmExtensionI);
        m_armExtensionMotorPidController.setD(kArmExtensionD);
        m_armExtensionMotorPidController.setFF(kArmExtensionFF);
        m_armExtensionMotorPidController.setIZone(kArmExtensionIzone);
        // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
        m_armExtensionMotorPidController.setSmartMotionMaxAccel(kSmartMaxAccel, 0);
        m_armExtensionMotorPidController.setSmartMotionMaxVelocity(kSmartMaxVel, 0);
        
        // TODO: try this out, but we don't want it on yet as it could hurt tuning
        // m_armExtensionMotorPidController.setSmartMotionAllowedClosedLoopError(kSmartAllowedError, 0);
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber(getName() + "/Arm Extension Ticks", getArmExtensionPosition());

        // SmartDashboard.putNumber(getName() + "/Arm Extension Applied Output", m_armExtensionMotor.getAppliedOutput());
        // SmartDashboard.putNumber(getName() + "/Arm Extension Output Current", m_armExtensionMotor.getOutputCurrent());
    }

    public void setArmExtensionPosition(double distance) {
        // m_armExtensionMotorPidController.setReference(distance, CANSparkMax.ControlType.kSmartMotion);
        m_armExtensionMotorPidController.setReference(distance, CANSparkMax.ControlType.kPosition);

        m_lastArmExtensionTarget = distance;
    }

    public void stopArmExtensionMotor() {
        setArmExtensionPosition(getArmExtensionPosition());
    }

    public boolean isArmExtensionAtSetpoint() {
        return Math.abs(m_lastArmExtensionTarget - getArmExtensionPosition()) < 5;
    }

    public double getArmExtensionPosition() {
        return m_extensionEncoder.getPosition();
    }

    public void smartDashboardInit() {
        SmartDashboard.putBoolean(getName() + "/Arm Extension Prof Const Enable", false);

        SmartDashboard.putNumber(getName() + "/Arm Extension P", m_armExtensionMotorPidController.getP());
        SmartDashboard.putNumber(getName() + "/Arm Extension I", m_armExtensionMotorPidController.getI());
        SmartDashboard.putNumber(getName() + "/Arm Extension D", m_armExtensionMotorPidController.getD());
        SmartDashboard.putNumber(getName() + "/Arm Extension FF", m_armExtensionMotorPidController.getFF());
        SmartDashboard.putNumber(getName() + "/Arm Extension Izone", m_armExtensionMotorPidController.getIZone());

        SmartDashboard.putNumber(getName() + "/Arm Extension Max Accel", kSmartMaxAccel);
        SmartDashboard.putNumber(getName() + "/Arm Extension Max Vel", kSmartMaxVel);
        SmartDashboard.putNumber(getName() + "/Arm Extension Closed Loop Error", 0);
    }

    public void smartDashboardUpdate() {
        m_armExtensionMotorPidController.setP(SmartDashboard.getNumber(getName() + "/Arm Extension P", kArmExtensionP));
        m_armExtensionMotorPidController.setI(SmartDashboard.getNumber(getName() + "/Arm Extension I", kArmExtensionI));
        m_armExtensionMotorPidController.setD(SmartDashboard.getNumber(getName() + "/Arm Extension D", kArmExtensionD));
        m_armExtensionMotorPidController.setFF(SmartDashboard.getNumber(getName() + "/Arm Extension FF", kArmExtensionFF));
        m_armExtensionMotorPidController.setIZone(SmartDashboard.getNumber(getName() + "/Arm Extension Izone", kArmExtensionIzone));

        if (SmartDashboard.getBoolean(getName() + "/Arm Extension Prof Const Enable", false))
        {
            var maxAccelExtension = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Accel", kSmartMaxAccel);
            var maxVelExtension = SmartDashboard.getNumber(getName() + "/Arm Rotator Max Vel", kSmartMaxVel);
            m_armExtensionMotorPidController.setSmartMotionMaxAccel(maxAccelExtension, 0);
            m_armExtensionMotorPidController.setSmartMotionMaxVelocity(maxVelExtension, 0);

            // var error = SmartDashboard.getNumber(getName() + "/Arm Rotator Closed Loop Error", 0);
            // TODO: enabled this for testing closed loop error
            // m_armExtensionMotorPidController.setSmartMotionAllowedClosedLoopError(error, 0);
        }

        SmartDashboard.putNumber(getName() + "/Arm Extension Velocity", m_extensionEncoder.getVelocity());
        SmartDashboard.putNumber(getName() + "/Arm Extension Position", m_extensionEncoder.getPosition());
    }
}
