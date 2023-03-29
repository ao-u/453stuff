//package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
public class ManualControlArmCmd extends CommandBase {
    private final ArmSubsystem m_armSubsystem;

    public ManualControlArmCmd(ArmSubsystem subsystem) {
        this.m_armSubsystem = subsystem;
        addRequirements(m_armSubsystem);
    }
    @Override
    public void execute() {
        m_armSubsystem.ControlArm();
    }
    @Override
    public void initialize() {

    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}
