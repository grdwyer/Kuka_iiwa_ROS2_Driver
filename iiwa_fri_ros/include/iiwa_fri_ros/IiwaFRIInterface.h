//
// Created by george on 21/09/17.
//

#ifndef IIWA_FRI_ROS_IIWAFRIINTERFACE_H
#define IIWA_FRI_ROS_IIWAFRIINTERFACE_H

#include <fri_client_sdk/friLBRClient.h>
#include "iiwa_fri_ros/IiwaHWInterface.h"
#include <memory>


class IiwaFRIInterface : public KUKA::FRI::LBRClient{
public:
    /*
     * Constructor
     */
    IiwaFRIInterface(std::shared_ptr<IiwaState> state);

    /*
     * Destructor
     */
    ~IiwaFRIInterface();

    /**
    * \brief Callback for FRI state changes.
    *
    * @param oldState
    * @param newState
    */
    virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

    /**
     * \brief Callback for the FRI state 'COMMANDING_ACTIVE'.
     */
    virtual void command();

    /**
     * \brief Callback for the FRI state MONITORING_{WAIT, READY}
     */
    virtual void monitor();

    /**
     * \brief Callback for the FRI state COMMANDING_WAIT
     */
    virtual void waitForCommand();

private:
    std::shared_ptr<IiwaState> iiwa_state_;
    // No control on the first loops to allow the robot to settle (prevents active control for approximately 200 ms at startup)
    bool init_ = false;
    int startup_ = 0;

    /**
     * \brief Helper function to update current values
     */
    void update_state();
};


#endif //IIWA_FRI_ROS_IIWAFRIINTERFACE_H
