package utilities;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;


/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class HomePosition extends RoboticsAPIApplication {

	public void initialize() {
		
	}
	
	public void run() {
		// Get robot
		LBR lbr = com.kuka.connectivity.userInterface.smartServo.ServoMotionUtilities.locateLBR(getContext()); 
		
		Tool tool = getApplicationData().createFromTemplate("Fetoscope");
		tool.attachTo(lbr.getFlange());
		
		//Get the fetoscope and move to the initial position.
		ObjectFrame Tip = tool.getFrame("/CalibrationTip/ShaftTip");
		Tip.move(ptp(getApplicationData().getFrame("/ScopeHome")).setJointVelocityRel(0.25));
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		HomePosition app = new HomePosition();
		app.runApplication();
	}
}
