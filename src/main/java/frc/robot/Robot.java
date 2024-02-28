package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import java.text.BreakIterator;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.AnalogInput;

public class Robot extends TimedRobot {
  // Compressor pCompressor = new Compressor(0,PneumaticsModuleType.CTREPCM);
  // DoubleSolenoid exampleDoublePCM =new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

  Timer timer;

  double time = Timer.getFPGATimestamp();
  double voltage_scale_factor = 1;
  // public DigitalOutput ultrasonicTriggerPinOne = new DigitalOutput(0);
  // public final AnalogInput ultrasonic = new AnalogInput(0);
  // private final AnalogInput ultrasonic = new AnalogInput(0);

  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  // private Joystick m_rightStick;

  private static final int leftDeviceID1 = 1;
  private static final int leftDeviceID2 = 2;
  private static final int rightDeviceID1 = 3;
  private static final int rightDeviceID2 = 4;

  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;

  private static PWMSparkMax atici1 = new PWMSparkMax(0);
  private static PWMSparkMax atici2 = new PWMSparkMax(1);
  private static PWMVictorSPX atici3 = new PWMVictorSPX(2);
  private static PWMVictorSPX atici4 = new PWMVictorSPX(3);
  // public static MotorControllerGroup firlatma = new
  // MotorControllerGroup(atici1,atici2);
  // private static PWMSparkMax asansor = new PWMSparkMax(2);
  // ultrasonic sensor

  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {

    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushed);
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushed);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushed);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushed);

    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();

    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);
    /*
     * m_leftMotor1.follow(m_rightMotor1);
     * m_rightMotor2.follow(m_rightMotor1);
     * // MotorControllerGroup rightMotorsGroupp = new
     * MotorControllerGroup(m_rightMotor1,m_rightMotor2);
     * // MotorControllerGroup leftMotorsGroupp = new
     * MotorControllerGroup(m_leftMotor1,m_leftMotor2);
     */
    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    m_leftStick = new Joystick(0);

    // leftMotorsGroupp.setInverted(false);
    // rightMotorsGroupp.setInverted(false);
    /*
     * leftMotor1.setSafetyEnabled(true);
     * leftMotor1.setExpiration(.1);
     * leftMotor1.feed();
     */
    m_myRobot.setSafetyEnabled(true);

  }

  @Override
  public void autonomousInit() {
    // BİR KERE ÇALIŞIR
    // System.out.println("Auto selected: ");
    timer = new Timer();
    timer.reset();
    timer.start();

    while (timer.get() < 5) {

      // atici1.set(-1);
      // atici2.set(-1);
      // atici3.set(-1);
      // atici4.set(-1);

      // m_myRobot.arcadeDrive(-m_leftStick.getTwist(), -m_leftStick.getY(), true);

    }
    timer.reset();

    while (timer.get() < 4) {
      // m_leftMotor1.set(.3);
      // m_rightMotor1.set(-.3);
    }
  }

  /** This function is called periodically during autonomous. */
  // Sürekli Çalışır
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("time", time);

    // Shooting sequence
    if (time < 2 && time > 0) {

      atici1.set(-1);
      atici2.set(-1);
      atici3.set(-1);
      atici4.set(-1);

      timer.stop();
      timer.reset();
      timer.start();

    }

    // After 2 seconds, stop shooting, as the note is probably gone
    if (time < 4 && time > 2) {
      atici1.set(0);
      atici2.set(0);
      atici3.set(0);
      atici4.set(0);
    }

    // After 4 seconds, move forward
    if (time < 6 && time > 4) {
      m_leftMotor1.set(.3);
      m_rightMotor1.set(-.3);
    }
    
    // After 6 seconds, stop
    if (time < 8 && time > 6) {
      m_leftMotor1.set(0);
      m_rightMotor1.set(0);
    }
  }

  @Override
  public void robotPeriodic() {
    // Publish range readings to SmartDashboard
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    m_myRobot.arcadeDrive(-m_leftStick.getTwist(), -m_leftStick.getY(), true);

    /*
     * if (m_leftStick.getRawButton(1)) {
     * exampleDoublePCM.set(Value.kForward);
     * } else if (m_leftStick.getRawButton(2)) {
     * exampleDoublePCM.set(Value.kReverse);
     * }
     */

    // asansor.set(m_stick.getRawAxis(1));

    if (m_leftStick.getRawButton(1)) {
      long start = System.currentTimeMillis();

      atici1.set(-1);
      atici3.set(-1);

      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace(); // Or handle the interruption in an appropriate way
      }

      atici2.set(-1);
      atici4.set(-1);
    }

    else if (m_leftStick.getRawButton(2)) {
      atici2.set(0.4);
      atici1.set(0.4);
      atici3.set(0.4);
      atici4.set(0.4);
    } else if (m_leftStick.getRawButton(4)) {
      atici2.set(-0.5);
      atici1.set(-0.5);
      atici3.set(-0.5);
      atici4.set(-0.5);
    }

    else {
      atici1.set(0);
      atici2.set(0);
      atici3.set(0);
      atici4.set(0);
    }

  }
}
