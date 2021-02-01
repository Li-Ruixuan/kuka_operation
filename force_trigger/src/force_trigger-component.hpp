#ifndef FORCE_TRIGGER_COMPONENT_HPP
#define FORCE_TRIGGER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <Eigen/Geometry>

namespace forcetrigger
{
/*
    Enter a description.
*/
class ForceTrigger : public RTT::TaskContext
    {
    public:

        ForceTrigger( std::string const& _name );
        bool configureHook( );
        bool startHook( );
        void updateHook( );
        void stopHook( );
        void cleanupHook( );

    protected:

        // Operations

        // Properties
        double prop_force_constraint_;
        double prop_probe_weight_;

        // Ports
        RTT::InputPort < std::vector < double > > in_W_ati_; 
        RTT::InputPort < std::vector < double > > in_fusion_;
        RTT::OutputPort < bool > out_contact_; 
        RTT::OutputPort < std::vector < double > > out_net_W_;
    private:

        // Member functions

        // Verified properties
        double force_constraint_;
        double probe_weight_;

        // Messages
        std::vector < double > msg_W_ati_; 
        std::vector < double > msg_fusion_;
        std::vector < double > msg_net_W_;

        // Flags
        bool   msg_contact_;
    };

} // End of namespace ForceTrigger.

#endif // End of ForceTriggerCOMPONENT
 
