package fri_controller;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

/**
 * Creates a FRI Session.
 */
public class FRIImpedanceController extends RoboticsAPIApplication
{
	private Controller lbr_controller_;
    private LBR lbr_;
    private String client_name_;
    private int send_period_, port_;
    private boolean motion_on_ = true;
    private Tool tool_;
    private double stiffness_, damping_;
    
    @Override
    public void initialize()
    {
        lbr_controller_ = (Controller) getContext().getControllers().toArray()[0];
        lbr_ = (LBR) lbr_controller_.getDevices().toArray()[0];
        tool_ = getApplicationData().createFromTemplate("EndEffector");
        tool_.attachTo(lbr_.getFlange());
        
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        client_name_ = getApplicationData().getProcessData("FRI_Client_IP").getValue();
        send_period_ = getApplicationData().getProcessData("FRI_Send_Period").getValue();
        port_ = getApplicationData().getProcessData("FRI_Port").getValue();
        
        stiffness_ = getApplicationData().getProcessData("joint_stiffness").getValue();
        damping_ = getApplicationData().getProcessData("joint_damping").getValue();
        
        setupUserKeys();
    }
    
    void setupUserKeys(){
		IUserKeyBar keybar = getApplicationUI().createUserKeyBar("Compliance Control");
		IUserKeyListener StopListener = new IUserKeyListener() {

			@Override
			public void onKeyEvent(IUserKey stopKey, UserKeyEvent event) {
				motion_on_ = false;
			}
		};

		IUserKey stopKey = keybar.addUserKey(0, StopListener, true);
		stopKey.setText(UserKeyAlignment.Middle, "Stop");
		stopKey.setLED(UserKeyAlignment.Middle, UserKeyLED.Green, UserKeyLEDSize.Small);

		keybar.publish();
	}

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(lbr_, client_name_);
        friConfiguration.setSendPeriodMilliSec(send_period_);
        friConfiguration.setPortOnRemote(port_);

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
	
	        // set to null motion
	        //PositionControlMode ctr_mode = new PositionControlMode();
	        JointImpedanceControlMode ctr_mode = new JointImpedanceControlMode(stiffness_, stiffness_, stiffness_, stiffness_, stiffness_, stiffness_, stiffness_);
	        ctr_mode.setDampingForAllJoints(damping_);
	        
	        PositionHold pos_hold = new PositionHold(ctr_mode, -1, null);
	        
	        IMotionContainer positionHoldContainer = tool_.moveAsync(pos_hold.addMotionOverlay(jointOverlay));
	        int i = 0;
	        while(motion_on_ == true){
	        	if(i % 30 == 0){
	        		getLogger().info(friSession.getFRIChannelInformation().toString());
	        	}
	        	ThreadUtil.milliSleep(100);
	        	i++;
	        }
	        
	        positionHoldContainer.cancel();
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
        final FRIImpedanceController app = new FRIImpedanceController();
        app.runApplication();
    }

}
