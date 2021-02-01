#ifndef VIRTUOSE6D_GUI_CONNECTOR_COMPONENT_HPP
#define VIRTUOSE6D_GUI_CONNECTOR_COMPONENT_HPP

#include <rtt/RTT.hpp>

namespace guiconnector
{
/*
    OROCOS compents that read a rostopic coming from the GUI and converts it to an OROCOS message usable for other components.
    It has only one Inputport and one OutputPort containing a string message with the GUI control type (such as btn_, chbx_, ...) and name.
    Other components can easily use this message by comparing the string value with an if condition.
*/
class GuiConnector : public RTT::TaskContext
    {
    public:

        GuiConnector( std::string const& _name );
        bool configureHook( );
        bool startHook( );
        void updateHook( );
        void stopHook( );
        void cleanupHook( );

    protected:

        // Operations

        // Properties

        // Ports
        RTT::InputPort < bool > in_ifnewpose_message_;
	RTT::InputPort < std::vector < double > > in_pose_message_;  
	RTT::OutputPort < std::vector < double > > out_pose_message_; 
        RTT::OutputPort < bool > out_write_finish_; 

    private:

        // Member functions

        // Verified properties

        // Messages
        std::vector < double > msg_pose_message_;  
        std::vector < double > msg_pose_message_out_;  
        // Flags
        bool   msg_write_finish_;
	bool   msg_ifnewpose_message_;
};

} // End of namespace guiconnector.

#endif // End of VIRTUOSE6D_GUI_CONNECTOR_COMPONENT
 
