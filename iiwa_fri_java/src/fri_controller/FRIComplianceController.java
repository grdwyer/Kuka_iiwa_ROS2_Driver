package fri_controller;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;

/**
 * Creates a FRI Session.
 */
public class FRIComplianceController extends RoboticsAPIApplication
{
	private Controller lbr_controller_;
    private LBR lbr_;
    private String client_name_;
    private int send_period_, port_;
    private boolean motion_on_ = true;
    private Tool tool_;
    private double transl_stiff_, rot_stiff_, transl_damp_, rot_damp_, null_stiff_, null_damp_;
    
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
        
        transl_stiff_ = getApplicationData().getProcessData("transl_stiffness").getValue();
        null_stiff_ = getApplicationData().getProcessData("null_stiffness").getValue();
        rot_stiff_ = getApplicationData().getProcessData("rot_stiffness").getValue();
        transl_damp_ = getApplicationData().getProcessData("transl_damping").getValue();
        null_damp_ = getApplicationData().getProcessData("null_damping").getValue();
        rot_damp_ = getApplicationData().getProcessData("rot_damping").getValue();
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

        PositionHold pos_hold;
        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        
	
	        getLogger().info("FRI connection established.");
	
	        // set to null motion
	        //PositionControlMode ctr_mode = new PositionControlMode();
	        CartesianImpedanceControlMode ctr_mode = new CartesianImpedanceControlMode();
	        ctr_mode.parametrize(CartDOF.TRANSL).setStiffness(transl_stiff_);
	        ctr_mode.parametrize(CartDOF.TRANSL).setDamping(transl_damp_);
	        
	        ctr_mode.parametrize(CartDOF.ROT).setStiffness(rot_stiff_);
	        ctr_mode.parametrize(CartDOF.ROT).setDamping(rot_damp_);
	        
	        ctr_mode.setNullSpaceStiffness(null_stiff_);
	        ctr_mode.setNullSpaceDamping(null_damp_);
	        
	        pos_hold = new PositionHold(ctr_mode, -1, null);
	        
	        IMotionContainer positionHoldContainer = tool_.moveAsync(pos_hold.addMotionOverlay(jointOverlay));
	        
	        int i = 0;
	        while(motion_on_ == true){
	        	if(i % 60 == 0){
	        		FRIChannelInformation info = friSession.getFRIChannelInformation();
	        		getLogger().info(String.format("Current Session: %s\nCurrent Jitter: %f\nCurrent Latency: %f\nCurrent Quality: %s", info.getFRISessionState().toString(), info.getJitter(), info.getLatency(), info.getQuality().toString()));
	        	}
	        	ThreadUtil.milliSleep(100);
	        	i++;
	        }
	        
	        friSession.close();
	        positionHoldContainer.cancel();
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            
        }
        catch (CommandInvalidException e){
        	getLogger().error(e.getLocalizedMessage());
            friSession.close();
            
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
        final FRIComplianceController app = new FRIComplianceController();
        app.runApplication();
    }

}
