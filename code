// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import java.util.Timer;
import edu.wpi.first.wpilibj.Solenoid;

// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


    // PNEUMATICS
  // Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  // initialized double solonoid object -- we use PCM to control -- we need to set toggle
  //https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#operating-pneumatic-cylinders
  // private final Compressor comp = new Compressor();
  private final Solenoid piston1 = new Solenoid(2, PneumaticsModuleType.CTREPCM, 0);
  private final Solenoid piston2 = new Solenoid(2, PneumaticsModuleType.CTREPCM, 1);

  Compressor pcmCompressor = new Compressor(2, PneumaticsModuleType.CTREPCM);

  // works -- motors and joystick
  TalonSRX rightMotor1 = new TalonSRX(13);
  TalonSRX rightMotor2 = new TalonSRX(14);
  TalonSRX rightMotor3 = new TalonSRX(15);
  TalonSRX leftMotor1 = new TalonSRX(10);
  TalonSRX leftMotor3 = new TalonSRX(12);
  TalonSRX funnyLeft = new TalonSRX(11);

  TalonSRX[] motors = {rightMotor1, rightMotor2, rightMotor3, leftMotor1, leftMotor3,  funnyLeft};

  TalonSRX gun = new TalonSRX(9);
  
    
  Joystick j = new Joystick(0);
  long autoStartTime; //time in ms
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    for (TalonSRX motor: motors) {
     motor.configPeakCurrentLimit(45); // once at 60 (40 previously) amps, limit current
     motor.configPeakCurrentDuration(80); // for 80ms 
     motor.configContinuousCurrentLimit(25   ); // set to 35 (20 previously) amps which will make robot slower
     motor.enableCurrentLimit(true); //enable 
     //motor.configVoltageCompSaturation(5); // value given is max voltage output of motors (maximum voltage output of robot battery is 13 V ish)
     //motor.enableVoltageCompensation(true);
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    this.autoStartTime = System.currentTimeMillis();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    long timePassed = System.currentTimeMillis() - autoStartTime;
    

    switch (m_autoSelected) { // part of template case kCustomAuto: // Put custom auto code here
      case kDefaultAuto: //1) shoot and intake case but not shoot again, shifts left
      double stickLeft = 0.2;
      double stickRight = -0.2;
      double srxIn = 0.8;
      double srxOut = -1;
        if (timePassed < 1000) {
          motorControl(false, -stickLeft);
          motorControl(true, stickRight);
          motorControl(false, stickLeft);
        } else if (timePassed < 2300) {
          gun.set(ControlMode.PercentOutput, 1);
        } else if (timePassed < 5500) {
          gun.set(ControlMode.PercentOutput, 0);
          motorControl(false, stickLeft);
          motorControl(true, -stickRight);
          motorControl(false, -stickLeft);
        } else if (timePassed < 8000) {
          motorControl(false, 0);
          motorControl(true, 0);
          motorControl(false, 0);
        }
        break;
      }
        /*
      if (timePassed < 1000) {
          rightMotors.set(ControlMode.PercentOutput, stickRight);
          funnyLeft.set(ControlMode.PercentOutput, -stickLeft);
          t0.set(ControlMode.PercentOutput, stickLeft); // forward
        } else if (timePassed < 2000) {
          // rightMotors.set(ControlMode.PercentOutput, stickRight);
          // funnyLeft.set(ControlMode.PercentOutput, stickLeft);
          // t0.set(ControlMode.PercentOutput, -stickLeft);
            exampleDoublePCM.set(Value.kForward);
            gun.set(ControlMode.PercentOutput, srxIn);
        } else if (timePassed < 5000) {
          rightMotors.set(ControlMode.PercentOutput, -stickRight);//turn
          funnyLeft.set(ControlMode.PercentOutput, -stickLeft);
          t0.set(ControlMode.PercentOutput, stickLeft);
        } else if (timePassed < 8000) { 
          rightMotors.set(ControlMode.PercentOutput, stickRight);
          funnyLeft.set(ControlMode.PercentOutput, -stickLeft);
          t0.set(ControlMode.PercentOutput, stickLeft);
        } else if (timePassed < 9000) {
          exampleDoublePCM.set(Value.kReverse);
          gun.set(ControlMode.PercentOutput, srxIn);
        } else if (timePassed < 12000) {
          rightMotors.set(ControlMode.PercentOutput, -stickRight);
          funnyLeft.set(ControlMode.PercentOutput, -stickLeft);
          t0.set(ControlMode.PercentOutput, stickLeft);
        } else if (timePassed < 15000) { 
          rightMotors.set(ControlMode.PercentOutput, stickRight);
          funnyLeft.set(ControlMode.PercentOutput, -stickLeft);
          t0.set(ControlMode.PercentOutput, stickLeft);
        } //can add a shoot command, but time is already up, and its safer for the driver to try and shoot from teleop
        break;
      }
    
    */
    //motor(leftDrive1, rightDrive1, 0.5, -0.5); // forward
  }

  /* This function is called once when teleop is enabled. */

  /* This function is called periodically during operator control. */
  public void motorControl(boolean x, double speed) {
    if (x) {
    rightMotor1.set(ControlMode.PercentOutput, speed);
    rightMotor2.set(ControlMode.PercentOutput, speed);
    rightMotor3.set(ControlMode.PercentOutput, speed);
    } else {
      leftMotor1.set(ControlMode.PercentOutput, speed);
      leftMotor3.set(ControlMode.PercentOutput, speed);
      funnyLeft.set(ControlMode.PercentOutput, -speed);
    }
  }

  @Override
  public void teleopInit() {
  }


  @Override
  public void teleopPeriodic() {
    double stickLeft = -j.getRawAxis(1);
    double stickRight = j.getRawAxis(5);
    double srxIn = j.getRawAxis(2);
    double srxOut = -j.getRawAxis(3);
    if (Math.abs(stickLeft) > 0.1){    
      motorControl(false, stickLeft);
    } else {motorControl(false, 0);}
    if (Math.abs(stickRight) > 0.1){
      motorControl(true, stickRight);
    } else {
      motorControl(true, 0);
    }

    if (Math.abs(srxIn) > 0.1) {
      gun.set(ControlMode.PercentOutput, srxIn*0.6);
    } else if (Math.abs(srxOut) > 0.1) {
      gun.set(ControlMode.PercentOutput, 1);
    }

     // PNEUMATICS
    if(j.getRawButton(4)) {
      piston1.set(true);
      piston2.set(false);
      // System.out.println("test1");
    } 
    if (j.getRawButton(3)) {
      piston1.set(false);
      piston2.set(true);
      // System.out.println("test2");
    }
    // System.out.println(pcmCompressor.enabled());
    pcmCompressor.enableDigital();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

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
