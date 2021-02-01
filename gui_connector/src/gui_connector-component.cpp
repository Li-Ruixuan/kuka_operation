#include "gui_connector-component.hpp"
#include <rtt/Component.hpp>
#include <unistd.h>
#include <rtt/RTT.hpp>
#include <Eigen/Geometry>

using namespace RTT;

namespace guiconnector
{
/*
    Constructor of the OROCOS component.
*/
GuiConnector::GuiConnector( std::string const& _name ) : TaskContext( _name, PreOperational )  
    , msg_pose_message_( 6, 0 ) 
    , msg_pose_message_out_( 6, 0 ) 
    , msg_write_finish_(false)
    , msg_ifnewpose_message_(false)
{
    // Add properties.

    // Add ports.
    addPort( "in_ifnewpose_message", in_ifnewpose_message_).doc( "the trigger message, new pose message is ready" );
    addPort( "in_pose_message", in_pose_message_ ).doc( "the position message send by ros, contains position data of next target point" );
    addPort( "out_pose_message", out_pose_message_ ).doc( " the output new pose message " );
    addPort( "out_write_finish", out_write_finish_ ).doc( "The converted messages to ros, to notice the target point has been fill in the trajectory." );

    // Add operations.

    // Show messages to the output ports to guarantee real-timeness.  
    out_write_finish_.setDataSample( msg_write_finish_ );  
    out_pose_message_.setDataSample( msg_pose_message_out_ );

    log( Info ) << "[" << getName( ) << "] Constructed" << endlog( );
} // End of GuiConnector constructor.

/*
    Configuration hook of the OROCOS component.
*/
bool GuiConnector::configureHook( )
{
    if ( msg_write_finish_ )
    {
        msg_write_finish_ = false;
    }
    if ( msg_ifnewpose_message_ )
    {
        msg_ifnewpose_message_ = false;
    }
    log( Info ) << "[" << getName( ) << "] Configured" << endlog( );

    return true;
} // End of configureHook.

/*
    Start hook of the OROCOS component.
*/
bool GuiConnector::startHook( )
{
    // Set output ports and initial msgs. 
    msg_write_finish_ = false ;
    out_write_finish_.write( msg_write_finish_ );
    std::fill( msg_pose_message_out_.begin( ), msg_pose_message_out_.end( ), 0 );
    out_pose_message_.write( msg_pose_message_out_);

    // Reset input ports.
    in_ifnewpose_message_.clear();
    in_pose_message_.clear();
    // Reset flags.

    log( Info ) << "[" << getName( ) << "] Started" << endlog( );

    return true;
} // End of startHook.

/*
    Update hook of the OROCOS component.
*/
void GuiConnector::updateHook( )
{
        // Read the message coming from the GUI and send it to other components.
    in_pose_message_.read( msg_pose_message_ ); 
    in_ifnewpose_message_.read( msg_ifnewpose_message_); 
    Eigen::Vector3d w_t_( msg_pose_message_[ 0 ],msg_pose_message_[ 1 ],msg_pose_message_[ 2 ] );
    Eigen::Vector3d w_o_( msg_pose_message_[ 3 ],msg_pose_message_[ 4 ],msg_pose_message_[ 5 ] );

    if ( msg_ifnewpose_message_ == true )
    {
      //sleep(1*1000);
      msg_write_finish_ = true;
    }
    else
    {
     msg_write_finish_ = false;
    }

    for ( int i = 0; i < 3; i++ )
    {
        msg_pose_message_out_ [i] = w_t_[i];
        msg_pose_message_out_ [i+3] = w_o_[i];
    }


    out_write_finish_.write( msg_write_finish_ );
    out_pose_message_.write( msg_pose_message_out_ ); 

} // End of updateHook.

/*
    Stop hook of the OROCOS component.
*/
void GuiConnector::stopHook( )
{
    msg_write_finish_ = false ;
    out_write_finish_.write( msg_write_finish_ );
    std::fill( msg_pose_message_out_.begin( ), msg_pose_message_out_.end( ), 0 );
    out_pose_message_.write( msg_pose_message_out_ );
  
    log( Info ) << "[" << getName( ) << "] Stopped" << endlog( );
} // End of stopHook.

/*
    Cleanup hook of the OROCOS component.
*/
void GuiConnector::cleanupHook( )
{
    log( Info ) << "[" << getName( ) << "] Cleaned up" << endlog( );
} // End of cleanupHook.

} // End of namespace guiconnector.

ORO_CREATE_COMPONENT( guiconnector::GuiConnector )
 
