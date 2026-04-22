package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final DoubleSolenoid climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimbConstants.climbForwardChannel, ClimbConstants.climbReverseChannel);
    
    public ClimbSubsystem(){
        
        compressor.enableDigital();

    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Climb/IsCompEnable", compressor.isEnabled());
        SmartDashboard.putBoolean("Climb/IsPressureReady", compressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Climb/ClimbReady", climbSolenoid.get() == DoubleSolenoid.Value.kForward);

    }

    public void climbForward(){
        climbSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void climbReverse(){
        climbSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void climbToggle() {
        
        if (climbSolenoid.get() == DoubleSolenoid.Value.kForward) {
            climbSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else {
            climbSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void compressorToggle(){
        compressor.disable();

        if(compressor.isEnabled()){
            compressor.disable();
        } else{
            compressor.enableDigital();
        }
    }

    public void compressorDisable(){
        compressor.disable();
    }

    public void compressorEnable(){
        compressor.enableDigital();
    }

    public boolean isCompReady(){
        return compressor.isEnabled();
    }



    

        
}
