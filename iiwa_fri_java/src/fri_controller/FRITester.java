package fri_controller;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;

/**
 * Creates a FRI Session.
 */
public class FRITester extends RoboticsAPIApplication
{
	private Controller lbr_controller_;
    private LBR lbr_;
    private String client_name_;
    private int send_period_;
    private Tool tool_;
    
    @Override
    public void initialize()
    {
        lbr_controller_ = (Controller) getContext().getControllers().toArray()[0];
        lbr_ = (LBR) lbr_controller_.getDevices().toArray()[0];
        tool_ = getApplicationData().createFromTemplate("ZR300");
        tool_.attachTo(lbr_.getFlange());
        
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        client_name_ = getApplicationData().getProcessData("FRI_Client_IP").getValue();
        send_period_ = getApplicationData().getProcessData("FRI_Send_Period").getValue();
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(lbr_, client_name_);
        friConfiguration.setSendPeriodMilliSec(send_period_);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        
	
	        getLogger().info("FRI connection established.");
	
	        tool_.move(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(-90), .0, Math.toRadians(0), .0));

	        // async move with overlay ...
	        tool_.moveAsync(ptp(Math.toRadians(-45), .0, .0, Math.toRadians(-90), .0, Math.toRadians(-90), .0)
	                .setJointVelocityRel(0.2)
	                .addMotionOverlay(jointOverlay)
	                .setBlendingRel(0.1)
	                );

	        // ... blending into sync move with overlay
	        tool_.move(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(-90), .0, Math.toRadians(0), .0)
	                .setJointVelocityRel(0.2)
	                .addMotionOverlay(jointOverlay)
	                );
	     // async move with overlay ...
	        tool_.moveAsync(ptp(Math.toRadians(-45), .0, .0, Math.toRadians(-90), .0, Math.toRadians(-90), .0)
	                .setJointVelocityRel(0.2)
	                .addMotionOverlay(jointOverlay)
	                .setBlendingRel(0.1)
	                );

	        // ... blending into sync move with overlay
	        tool_.move(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(-90), .0, Math.toRadians(0), .0)
	                .setJointVelocityRel(0.2)
	                .addMotionOverlay(jointOverlay)
	                );
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }
        finally{
        // done
        	friSession.close();
        }
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final FRITester app = new FRITester();
        app.runApplication();
    }

}
