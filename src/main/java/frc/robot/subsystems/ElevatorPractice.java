/** 
package frc.robot.subsystems;
import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
//this may or may not work
public class Elevator extends SubsystemBase{

  // Define the motor and encoders
  private final SparkMax elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevator, MotorType.kBrushless);
  private SparkClosedLoopController elevatorPid = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();


  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(2.54, 2.54);
  private final TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;

 
 public Elevator() {


  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  
 // TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.54, 2.54);
  //TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(0, 0.0);
 // TrapezoidProfile profile = new TrapezoidProfile(constraints);


    config
        .inverted(true)
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);
    //config.signals.primaryEncoderPositionPeriodMs(5);
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       //double position = elevatorMotor.getEncoder().getPosition();
    
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pid(Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD);

   // elevatorPid.setReference(Constants.ElevatorConstants.KP, null);
   // elevatorPid.setReference(Constants.ElevatorConstants.KI, null);
   // elevatorPid.setReference(Constants.ElevatorConstants.KD, null);
    //elevatorPid.setReference(Constants.ElevatorConstants.KFF, null);

   
   elevatorMotor.getEncoder();

    config.encoder.positionConversionFactor(Constants.ElevatorConstants.kMeterPerRevolution);
    //config.closedLoop.setFeedbackDevice(RelativeEncoder);
  

   // elevatorPid.setFeedbackDevice(); 
   


  }

  protected void useState(TrapezoidProfile.State m_profile) {
    elevatorPid.setReference(m_profile.position, ControlType.kPosition);
    //elevatorPid.setReference(Constants.ElevatorConstants.KP, ControlType.kPosition);
  }

  public Command setElevatorGoalCommand(double goal) {
    m_goal = new TrapezoidProfile.State(goal, m_constraints.maxVelocity);
    return Commands.runOnce(() -> setGoal(m_goal), this);
  }

   
  public double getEncoder(){
    return elevatorEncoder.getPosition();
  }

  private void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  @Override
  public void periodic() {
    // Display current values on the SmartDashboard

    SmartDashboard.putNumber("Elevator Raw", getEncoder());
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());

    
  }
}
*/

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;


public class ElevatorPractice extends SubsystemBase {

 
  private static ElevatorPractice mInstance;
  private PeriodicIO mPeriodicIO;

  

  public static ElevatorPractice getInstance() {
    if (mInstance == null) {
      mInstance = new ElevatorPractice();
    }
    return mInstance;
  }

  private SparkMax mLeftMotor;
  private RelativeEncoder mLeftEncoder;
  private SparkClosedLoopController mLeftPIDController;

  private SparkMax mRightMotor;
  private RelativeEncoder mRightEncoder;
  private SparkClosedLoopController mRightPIDController;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  private ElevatorPractice() {
    super("Elevator");

    mPeriodicIO = new PeriodicIO();

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.closedLoop
    .pid(Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD)
        .minOutput(Constants.ElevatorConstants.kMinOutput)
        .maxOutput(Constants.ElevatorConstants.MaxOutput);

    elevatorConfig.smartCurrentLimit(40);

    elevatorConfig.idleMode(IdleMode.kBrake);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new SparkMax(Constants.ElevatorConstants.elevator, MotorType.kBrushless);
    mRightEncoder = mRightMotor.getEncoder();
    mRightPIDController = mRightMotor.getClosedLoopController();
    mRightMotor.configure(
        elevatorConfig.follow(mLeftMotor),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.54, 2.54);
        TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(0, 0.0);
        TrapezoidProfile profile = new TrapezoidProfile(constraints);
  }

  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // TODO: Use this pattern to only drive slowly when we're really high up
    // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
    // mPeriodicIO.is_pivot_low = true;
    // } else {
    // mPeriodicIO.is_pivot_low = false;
    // }
  }

  
  /*public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = mPeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
     /*  mLeftPIDController.setReference(
          mCurState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
         // Constants.ElevatorConstants.kG,
          ArbFFUnits.kVoltage);
    } else {
      mCurState.position = mLeftEncoder.getPosition();
      mCurState.velocity = 0;
      mLeftMotor.set(mPeriodicIO.elevator_power);
    }
  }
    */

  
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }


  
  public void reset() {
    mLeftEncoder.setPosition(0.0);
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    //putNumber("setElevatorPower", power);
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    mPeriodicIO.is_elevator_pos_control = true;
   // mPeriodicIO.elevator_target = Constants.ElevatorConstants.kLevel3;
    mPeriodicIO.state = ElevatorState.STOW;
  }

 
  
  /*---------------------------------- Custom Private Functions ---------------------------------*/
}