/*
  2022 everybot code
  written by carson graf 
  don't email me, @ me on discord
*/

/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

  // Definitions for the hardware. Change this if you change what stuff you have
  // plugged in
  CANSparkMax driveLeftA = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveLeftB = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveRightA = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

  Joystick driverController = new Joystick(0);
  Joystick gunnerController = new Joystick(1);

  WPI_Pigeon2 gyro = new WPI_Pigeon2(0);
  SendableChooser<String> autoChooser = new SendableChooser<String>();

  // Constants for controlling the arm. consider tuning these for your particular
  // robot
  final double armHoldUp = 0.08;
  final double armHoldDown = 0.13;
  final double armTravel = 0.7;

  final double armTimeUp = 0.3;
  final double armTimeDown = 0.2;

  // Varibles needed for the code
  boolean armUp = true; // Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0;

  double autoStart = 0;
  boolean goForAuto = false;
  String autoType = "";

  double kP;
  double kI;
  double kD;
  double levelingTolerance;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Configure motors to turn correct direction. You may have to invert some of
    // your motors
    driveLeftA.setInverted(false);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(false);
    driveLeftB.burnFlash();
    driveRightA.setInverted(true);
    driveRightA.burnFlash();
    driveRightB.setInverted(true);
    driveRightB.burnFlash();

    arm.setInverted(false);
    arm.setIdleMode(IdleMode.kBrake);
    arm.burnFlash();

    // add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

    SmartDashboard.putNumber("kP Value", 0);
    kP = SmartDashboard.getNumber("kP Value", 0);

    SmartDashboard.putNumber("kI Value", 0);
    kI = SmartDashboard.getNumber("kI Value", 0);

    SmartDashboard.putNumber("kD Value", 0);
    kD = SmartDashboard.getNumber("kD Value", 0);

    SmartDashboard.putNumber("Leveling Tolerance", 0);
    levelingTolerance = SmartDashboard.getNumber("Leveling Tolerance", 0);

    autoChooser.addOption("Cube", "Cube");
    autoChooser.addOption("Cone", "Cone");
    autoChooser.addOption("Just Move", "Just Move");

    SmartDashboard.putData(autoChooser);

    // TODO mount the camera, stream is already setup
    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    autoType = autoChooser.getSelected();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // arm control code. same as in teleop
    if (armUp) {
      if (Timer.getFPGATimestamp() - lastBurstTime < armTimeUp) {
        arm.set(armTravel);
      } else {
        arm.set(armHoldUp);
      }
    } else {
      if (Timer.getFPGATimestamp() - lastBurstTime < armTimeDown) {
        arm.set(-armTravel);
      } else {
        arm.set(-armHoldUp);
      }
    }

    // get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if (goForAuto) {
      // series of timed events making up the flow of auto
      if (autoTimeElapsed < 3) {
        // spit out the ball for three seconds
        intake.set(-.7);
      } else if (autoTimeElapsed < 6) {
        // stop spitting out the ball and drive backwards *slowly* for three seconds
        intake.set(0);
        driveLeftA.set(-0.3);
        driveLeftB.set(-0.3);
        driveRightA.set(-0.3);
        driveRightB.set(-0.3);
      } else {
        // do nothing for the rest of auto
        intake.set(0);
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    armUp = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Set up arcade steer
    double forward;
    double turn;
    // double speedCap;

    // if (driverController.getRawButton(5)) { // Rabbit
    //   speedCap = 1;
    // } else if (driverController.getRawButton(6)) { // Turtle
    //   speedCap = .4;
    // } else { // Default
    //   speedCap = .8;
    // }

    forward = MathUtil.applyDeadband(-driverController.getRawAxis(5), .1);
    // if(forward > speedCap){
    //   forward = speedCap;
    // }

    turn = MathUtil.applyDeadband(-driverController.getRawAxis(0), .1);
    if (turn > 0.5) {
      turn = 0.5;
    }

    double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;

    driveLeftA.set(driveLeftPower);
    driveLeftB.set(driveLeftPower);
    driveRightA.set(driveRightPower);
    driveRightB.set(driveRightPower);

    // Intake controls
    if (gunnerController.getRawButton(5)) {
      intake.set(1);
      ;
    } else if (gunnerController.getRawButton(7)) {
      intake.set(-1);
    } else {
      intake.set(0);
    }

    // Arm Controls
    if (armUp) {
      if (Timer.getFPGATimestamp() - lastBurstTime < armTimeUp) {
        arm.set(armTravel);
      } else {
        arm.set(armHoldUp);
      }
    } else {
      if (Timer.getFPGATimestamp() - lastBurstTime < armTimeDown) {
        arm.set(-armTravel);
      } else {
        arm.set(-armHoldDown);
      }
    }

    if (gunnerController.getRawButtonPressed(6) && !armUp) {
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
    } else if (gunnerController.getRawButtonPressed(8) && armUp) {
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
    }

    SmartDashboard.putNumber("Gyro", gyro.getPitch());
  }

  @Override
  public void disabledInit() {
    // On disable turn off everything
    // done to solve issue with motors "remembering" previous setpoints after
    // reenable
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    arm.set(0);
    intake.set(0);
  }

  /**
   * Sets all drive motors to set speed
   *
   * @param speed Speed of motors.
   * 
   */
  public void drive(double speed) {
    driveLeftA.set(speed);
    driveLeftB.set(speed);
    driveRightA.set(speed);
    driveRightB.set(speed);
  }

  /**
   * Attempts to level the robot on the platform
   */
  public void attemptLevel() {

    kP = SmartDashboard.getNumber("kP Value", 0);
    kI = SmartDashboard.getNumber("kI Value", 0);
    kD = SmartDashboard.getNumber("kD Value", 0);
    levelingTolerance = SmartDashboard.getNumber("Leveling Tolerance", 0);

    PIDController leveler = new PIDController(kP, kI, kD);

    leveler.setTolerance(levelingTolerance);

    while (leveler.atSetpoint() != false) {
      drive(leveler.calculate(gyro.getPitch(), 0));
    }

    leveler.close();
  }

}