package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO
// - set up the PID loop
// - set up encoder conversions

// can probably take some things out of the subsystem. Copied from frc 868 team
public class Elevator extends SubsystemBase {
    private final SparkMax m_elevator;
    private final RelativeEncoder m_encoder;
    private final ProfiledPIDController m_controller;

    
    private boolean overrideFlag = false;
    private double outputSpeed = 0;
    
    public Elevator() {
        // TODO: Tune trapezoid profile
        m_controller = new ProfiledPIDController(Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KP, new TrapezoidProfile.Constraints(2.54 , 2.54));
        
        m_elevator = new SparkMax(Constants.ElevatorConstants.elevator, MotorType.kBrushless);
        m_encoder = m_elevator.getEncoder();
       

        configMotor();

        m_encoder.setPosition(0);
        
        m_controller.reset(0);
    }

    /**
     * Set the position of the elevator
     * @param pos the position in inches
     */
    public void setPositionGoal(double pos) {
        // TODO: Remove this check as redundance with enableContinuousInput?
        if (pos > Constants.ElevatorConstants.maxHeight)
        {
            System.out.printf("cannot set position to height %f - max height is %f!\n", pos, Constants.ElevatorConstants.maxHeight);
        }
        else
        {
            m_controller.setGoal(pos);
        }
    }

    public void manual(double speed){
        m_elevator.set(speed);
    }

    public void overrideUp()
    {
        outputSpeed = Constants.ElevatorConstants.elevatorSpeed;
        overrideFlag = true;
    }

    public void overrideDown()
    {
        outputSpeed = -Constants.ElevatorConstants.elevatorSpeed;
        overrideFlag = true;
    }

    public void stop()
    {
        outputSpeed = 0;
        overrideFlag = false;
    }

    public double getPosition()
    {
        return m_encoder.getPosition();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Elevator Raw", m_controller.calculate(m_encoder.getPosition()));

    }

    private void configMotor() {
        var config = new SparkMaxConfig();

        config.idleMode(Constants.ElevatorConstants.kIdleMode);
        config.encoder // TODO: Setup the desired units we are using <---- DO NOT FORGET UNITS
            .positionConversionFactor((((Units.inchesToMeters(1.9) * Math.PI) / 25)))
            .velocityConversionFactor((((Units.inchesToMeters(1.9) * Math.PI) / 25) / 60)); // in meters per second
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pidf(
                Constants.ElevatorConstants.KP,
                Constants.ElevatorConstants.KI,
                Constants.ElevatorConstants.KD,
                Constants.ElevatorConstants.KFF
            );
        // config.smartCurrentLimit(60, 30);
        config.inverted(true); // True is the correct way

        m_elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}
