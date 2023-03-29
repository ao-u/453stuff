//package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
public class RetractArmCmd extends CommandBase {
    private final ArmSubsystem m_armSubsystem;

    public RetractArmCmd(ArmSubsystem subsystem) {
        this.m_armSubsystem = subsystem;
        addRequirements(m_armSubsystem);
    }
    @Override
    public void execute() {
        
    }
    @Override
    public void initialize() {
        m_armSubsystem.RetractArm();
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}