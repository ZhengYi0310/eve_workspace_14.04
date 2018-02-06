/*************************************************************************
	> File Name: biotac_finger_interface.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Fri 30 Jun 2017 10:19:14 AM PDT
 ************************************************************************/

#ifndef _BIOTAC_HAND_INTERFACE_H
#define _BIOTAC_HAND_INTERFACE_H
//#include <cstdint>

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <biotac_sensors/biotac_hand_class.h>
#include <biotac_sensors/BioTacHand.h>
#include <biotac_sensors/BioTacData.h>
#include <biotac_sensors/BioTacTime.h>

#include <cassert>
#include <string>

#define NUM_PAC_DATA 22
#define NUM_ELECTRODE_DATA 19

namespace barrett_model
{
    /** A handle used to read the state of a biotac sensor or a hand ??. */
    class BiotacHandStateHandle 
    {
        public:
            BiotacHandStateHandle() : bt_hand_id_(""), bt_hand_(NULL)
            {}; //TODO:: think about if add biotac time info 

            
            BiotacHandStateHandle(const std::string& bt_hand_id, const biotac::BioTacHandClass* bt_hand) : bt_hand_id_(bt_hand_id)
            {
                if (!bt_hand)
                {
                    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + bt_hand_id + "'. Biotac hand class pointer is null.");
                }

                bt_hand_ = bt_hand;
            }

            std::string getName() const {return bt_hand_id_;}
			
			biotac::BioTacHandClass getBioTacHand() const {assert(bt_hand_); return *bt_hand_;}

        private:
            std::string bt_hand_id_;
			const biotac::BioTacHandClass* bt_hand_;
    };

    /** \brief Hardware interface to support reading the state of an array of joints
     *
     * This \ref HardwareInterface supports reading the state of an array of named
     * joints, each of which has some position, velocity, and effort (force or
     * torque).
     *
     */
     class BiotacHandStateInterface : public hardware_interface::HardwareResourceManager<BiotacHandStateHandle> {};
}
#endif
