package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Kinematics;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.classes.SmoothControl;
import frc.robot.classes.TargetPosition2D;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.AutonomousCommandConstants.*;

public class DriveTo extends CommandBase{

    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    private TargetPosition2D m_targetPose;
    private SmoothControl m_smoothControl;
    private boolean m_isDone = false;

    public DriveTo(TargetPosition2D targetPose, Kinematics kinematics, Drivetrain drivetrain) {
        addRequirements(drivetrain);
        m_targetPose = targetPose;
        m_kinematics = kinematics;
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        //Content to be run when initializing the command
        
        //Initialize smooth control
        m_smoothControl = new SmoothControl();
        m_smoothControl.reset();
    }

    @Override
    public void execute() {
        double vR = 0.0;
        double vL = 0.0;

        //Start auto nav drive routine
        //Compute turn rate and update range
        m_smoothControl.computeTurnRate(m_kinematics.getPose(), m_targetPose, m_drivetrain.getRobotVelocity());
            
        try {
            //Calculate vR in feet per second
            vR = m_targetPose.getVelocity() - (TRACK_WIDTH_FEET/2)*m_smoothControl.getTurnRateRadians();
            //Calculate vL in feet per second
            vL = m_targetPose.getVelocity() + (TRACK_WIDTH_FEET/2)*m_smoothControl.getTurnRateRadians();
        } 
        catch (Exception e) {
            System.out.println("AutonomousCommand ERROR: Failed to retrieve pose velocity " + (m_targetPose.getVelocity()));
            m_isDone = true;
        }
        
        SmartDashboard.putNumber("Desired Left Velocity (ft/s)", vL);
        SmartDashboard.putNumber("Desired Right Velocity (ft/s)", vR);
        SmartDashboard.putNumber("Auto Range", m_smoothControl.getRange());
        SmartDashboard.putNumber("Auto Omega Desired (Degrees)", m_smoothControl.getTurnRateDegrees());
        SmartDashboard.putString("Next Target", m_targetPose.getX() + ", "+ m_targetPose.getY() + ", "+ m_targetPose.getHeadingDegrees());

        //Converting ft/s equation output to controller velocity
        vR = SPIKE293Utils.feetPerSecToControllerVelocity(vR);
        vL = SPIKE293Utils.feetPerSecToControllerVelocity(vL);

        //Send vR and vL to velocity drive, units are in controller velocity
        m_drivetrain.velocityDrive(vL, vR);

        //Have we reached the target?
        if(TARGET_WITHIN_RANGE_FEET >= m_smoothControl.getRange()) {
            //ending the command to allow the next sequential command with next point to run
            m_isDone = true;
        }

    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}