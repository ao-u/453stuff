

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ExtendArmCmd;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase{
    private final WPI_TalonSRX armRotationMotor = new WPI_TalonSRX(Constants.IOConstants.talonFXPort);
    private final WPI_TalonSRX armExtensionMotor = new WPI_TalonSRX(Constants.IOConstants.talonFXPort);
    public DoubleSolenoid m_clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.kForwardChannel, ArmConstants.kReverseChannel);
    private Joystick operJoystick = new Joystick(Constants.IOConstants.kOperatorControllerPort);
    public ArmSubsystem() {
    }
    public void CloseClaw() {
        m_clawSolenoid.set(kReverse);
    }
    public void OpenClaw() {
        m_clawSolenoid.set(kForward);
    }
    public void ControlArm() {
        armRotationMotor.set(ControlMode.PercentOutput, operJoystick.getY());
    }
    public void ExtendArm() {
        armExtensionMotor.set(ControlMode.Position, Constants.ArmConstants.kExtended);
    }
    public void RetractArm() {
        armExtensionMotor.set(ControlMode.Position, Constants.ArmConstants.kRetracted);
    }
}
