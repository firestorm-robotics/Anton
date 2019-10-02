/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Utils.Constants;
import frc.robot.commands.CartesianDrive;
import frc.robot.commands.curvatureDriveCommand;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax mMasterLeft  = new CANSparkMax(RobotMap.leftMaster, MotorType.kBrushless);
  private CANSparkMax mLeftSlaveA  = new CANSparkMax(RobotMap.leftSlaveA, MotorType.kBrushless);
  private CANSparkMax mMasterRight = new CANSparkMax(RobotMap.rightMaster,MotorType.kBrushless);
  private CANSparkMax mRightSlaveA = new CANSparkMax(RobotMap.rightSlaveA,MotorType.kBrushless);
  private final double HIGH_GEAR_RATIO = 9.74;
  private  final double WHEEL_RADIUS = 3;
  private final double WHEEL_CIRCUMFERENCE = Math.PI*(2*WHEEL_RADIUS);
  private  final double INCHES_PER_REV = WHEEL_CIRCUMFERENCE/HIGH_GEAR_RATIO;
  private double m_quickStopAccumulator = 0;
  private double m_quickStopAlpha = 0.1;



  private CANPIDController mPIDLeft  = new CANPIDController(mMasterLeft);
  private CANPIDController mPIDRight = new CANPIDController(mMasterRight);
  private CANEncoder mEncoderLeft = new CANEncoder(mMasterLeft);
  private CANEncoder mEncoderRight = new CANEncoder(mMasterRight);
  public double kP,kD,kI,maxVel, maxAcc,kFF,kMaxOutput, kMinOutput, kIz, maxRPM;
  private double displacement;
  private static Drivetrain instance;

  private Drivetrain()
  {
    mLeftSlaveA.follow(mMasterLeft);
    mRightSlaveA.follow(mMasterRight);

    mMasterRight.setInverted(true);
    mRightSlaveA.setInverted(true);
    kP = .005;
    kI = .0000096;
    kD = .0005;
    kIz = 0;
    kFF = 0.00000999;
    maxRPM = 700;
    maxVel = 10000;
    maxAcc = 500;
    kMaxOutput = 1;
    kMinOutput = -1;
    mPIDLeft.setP(kP);
    mPIDRight.setP(kP);
    mPIDLeft.setI(kI);
    mPIDRight.setI(kI);
    mPIDLeft.setD(kD);
    mPIDRight.setD(kD);
    mPIDLeft.setFF(kFF);
    mPIDRight.setFF(kFF);
    mPIDLeft.setIZone(kIz);
    mPIDRight.setIZone(kIz);
    int smartMotionSlot = 0;
    mPIDLeft.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    mPIDRight.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    mPIDLeft.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    mPIDRight.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    mPIDLeft.setOutputRange(-1, 1);
    mPIDRight.setOutputRange(-1, 1);

    mMasterLeft.setSmartCurrentLimit(12);
    mMasterRight.setSmartCurrentLimit(12);
    mLeftSlaveA.setSmartCurrentLimit(12);
    mRightSlaveA.setSmartCurrentLimit(12);

    mPIDLeft.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    mPIDRight.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    //mPIDLeft.setSmartMotionMinOutputVelocity(minVel, 0);
    //mPIDRight.setSmartMotionMinOutputVelocity(minVel, 0);
    mMasterLeft.setOpenLoopRampRate(0.25);
    mMasterRight.setOpenLoopRampRate(0.25);

    //mMasterLeft.setClosedLoopRampRate(0.5);
    //mMasterRight.setClosedLoopRampRate(0.5);
  }

 /**
   * @return the instance
   */
  public static Drivetrain getInstance() {
    if(instance == null)
    {
      instance = new Drivetrain();
    }
    return instance;
  }


  public void cartesianDrive(double y, double x)
  {
    y = Constants.applyDeadband(y,0.12);
    x = Constants.applyDeadband(x, 0.05);
    double tempX = (Constants.kDriveSensFactor*Math.pow(x,3)) + ((1 - Constants.kDriveSensFactor)*x);
    double tempY = (Constants.kDriveSensFactor*Math.pow(y,3)) + ((1 - Constants.kDriveSensFactor)*y);

    double powerLeft  = tempY - tempX;
    double powerRight = tempY + tempX;

    System.out.println("X:" + powerLeft);
    System.out.println("Y:" + y);
    mPIDLeft.setReference(Constants.clamp(powerLeft, -1, 1) , ControlType.kDutyCycle);
    mPIDRight.setReference(Constants.clamp(powerRight,-1,1), ControlType.kDutyCycle);
  }

  public void curvatureDrive(double xSpeed, double y, boolean isQuickTurn)
  {
    boolean overPower = false;
    double angularPow =0;
    xSpeed = Constants.applyDeadband(xSpeed, 0.12);
    y = Constants.applyDeadband(y, Constants.kDriveDeadBand);
    System.out.println(xSpeed);
    if(isQuickTurn)
    {
       if(Math.abs(xSpeed) < 0.2)
       {
        m_quickStopAccumulator = (1 - m_quickStopAlpha)*m_quickStopAccumulator + m_quickStopAlpha * Constants.clamp(y, -1.0, 1.0)*2;
       }
       overPower = true;
       angularPow = y;
    }else
    {
      overPower = false;
      angularPow = Math.abs(xSpeed) * y - m_quickStopAccumulator;
      System.out.println("APow:"+m_quickStopAccumulator);

      if(m_quickStopAccumulator > 1)
      {
        m_quickStopAccumulator -= 1;
      }else if(m_quickStopAccumulator < -1)
      {
        m_quickStopAccumulator += 1;
      }else
      {
        m_quickStopAccumulator = 0.0;
      }

      
    }

    double leftPow = xSpeed - angularPow;
    double rightPow = xSpeed + angularPow;

    if (overPower) {
      if (leftPow > 1.0) {
        rightPow -= leftPow - 1.0;
        leftPow = 1.0;
      } else if (rightPow > 1.0) {
        leftPow -= rightPow - 1.0;
        rightPow = 1.0;
      } else if (leftPow < -1.0) {
        rightPow -= leftPow + 1.0;
        leftPow = -1.0;
      } else if (rightPow < -1.0) {
        leftPow -= rightPow + 1.0;
        rightPow = -1.0;
      }


    //System.out.println("L:"+ leftPow);
  }

  mPIDLeft.setReference(leftPow, ControlType.kDutyCycle);
  mPIDRight.setReference(rightPow, ControlType.kDutyCycle);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new curvatureDriveCommand());
  }
  public void resetEncoders()
  {
    mEncoderLeft.setPosition(0);
    mEncoderRight.setPosition(0);
  }
  public void testAutonomous () {
    double feet = 12/INCHES_PER_REV;
    //get displacement
    displacement = (Math.abs(mEncoderLeft.getPosition()+Math.abs(mEncoderRight.getPosition()))/2);
    double velocity = (Math.abs(mEncoderLeft.getVelocity()+Math.abs(mEncoderRight.getVelocity()))/2);
    System.out.println("Displacement: "+displacement+" Velocity: "+velocity);
    if(displacement < 10)
    {
      setVelocity(0.03, 0.03);
      System.out.println("Stopping");
    }else
    {
      setVelocity(0,0);
    }
  }

  private void setVelocity(double left, double right)
  {
    mPIDLeft.setReference(left,ControlType.kVelocity);
    mPIDRight.setReference(right,ControlType.kVelocity);
  }

  
}
