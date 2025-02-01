package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeIn extends InstantCommand {
    private final Intake m_intake;
     public IntakeIn(Intake intake) {
         m_intake = intake;
         addRequirements(m_intake);
    }
    
}
