package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import  com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Intake extends SubsystemBase {

    private final SparkMax intakeLeftMotor = new SparkMax(Constants.IntakeConstants.intakeLeft, MotorType.kBrushless);
    private SparkMax intakeRightMotor = new SparkMax(Constants.IntakeConstants.intakeRight, MotorType.kBrushless);
    private  SparkMax intakeChooserMotor = new SparkMax(Constants.IntakeConstants.intakeChooser, MotorType.kBrushless);
    private  TimeOfFlight trampSensor = new TimeOfFlight(0);
    private  TimeOfFlight shootSensor = new TimeOfFlight(1);
    private final  SparkFlex shooterFeederMotor = new SparkFlex(Constants.IntakeConstants.shooterFeeder, MotorType.kBrushless);

public Intake() {


    configureSparkFlex(shooterFeederMotor);
        
    }
    
    
    private void configureSparkFlex(SparkFlex shooterFeederMotor) {
        SparkFlexConfig config = new SparkFlexConfig();
        config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(100);
        shooterFeederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        

      
    }
    
    
    private void configureSparkMax( SparkMax intakeLeftMotor, boolean reverse) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(30);
    intakeLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    
    intakeRightMotor = new SparkMax(Constants.IntakeConstants.intakeRight, MotorType.kBrushless);
    config
  .smartCurrentLimit(30)
    .idleMode(IdleMode.kBrake)
   .follow(intakeLeftMotor,true);

    intakeChooserMotor = new SparkMax(Constants.IntakeConstants.intakeChooser, MotorType.kBrushless);
    config
    .smartCurrentLimit(30)
   .idleMode(IdleMode.kBrake);

   intakeChooserMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    trampSensor = new TimeOfFlight(0);
    trampSensor.setRangingMode(RangingMode.Short, 24);
    shootSensor = new TimeOfFlight(1);
    shootSensor.setRangingMode(RangingMode.Short, 24);

    //Feeder Motor
   /*  shooterFeederMotor = new SparkFlex(Constants.IntakeConstants.shooterFeeder, MotorType.kBrushless);
    config 
    .smartCurrentLimit(100)
   .idleMode(IdleMode.kBrake);
   */
  }
  public void runIntakeSpeed(double speed){
    intakeLeftMotor.set(speed);  
  }

  public void runChooserSpeed(double speed){
    intakeChooserMotor.set(speed);
  }

  public double getTrampDistance(){
    return trampSensor.getRange();
  }

  public double getShooterDistance(){
    return shootSensor.getRange();
  }

  public void runFeederSpeed(double speed){
    shooterFeederMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tramp", getTrampDistance());
    SmartDashboard.putNumber("Shooter", getShooterDistance());
  }

}







    
    



