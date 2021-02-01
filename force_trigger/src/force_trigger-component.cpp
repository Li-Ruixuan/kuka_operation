#include "force_trigger-component.hpp"
#include <rtt/Component.hpp>
#include <math.h>
#include <Eigen/Eigen>  
#include <stdlib.h>  
#include <Eigen/Geometry>  
#include <Eigen/Core>  
#include <vector>   
#include <iomanip> 
#include <stdio.h>
#include <cmath>

using namespace Eigen;
using namespace RTT;
using namespace std; 

namespace forcetrigger
{
/*
    Constructor of the OROCOS component.
*/
ForceTrigger::ForceTrigger( std::string const& _name ) : TaskContext( _name, PreOperational )
    , prop_force_constraint_( 0.0 ) 
    , prop_probe_weight_( 0.5) 
    , msg_W_ati_( 6, 0 ) 
    , msg_fusion_( 6, 0 )
    , msg_net_W_( 6, 0 )
    , msg_contact_( false ) 
{
    // Add properties.
    addProperty( "force_constraint", force_constraint_ ).doc( "The limit contact force of F/T sensor" );
    addProperty( "probe_weight", probe_weight_ ).doc( "The weight of us probe" );

    // Add ports.
    addPort( "in_W_ati", in_W_ati_ ).doc( "The wrench measured by the ATI sensor." );
    addPort( "in_fusion", in_fusion_ ).doc( "The wrench measured by the ATI sensor." );
    addPort( "out_contact", out_contact_ ).doc( "The output message of contact force" );
    addPort( "out_net_W", out_net_W_ ).doc( "The wrench measured by the ATI sensor after gravity compenzation." );
    
    // Add operations.

    // Show messages to the output ports to guarantee real-timeness.
    out_contact_.setDataSample( msg_contact_ ); 
    out_net_W_.setDataSample( msg_net_W_ ); 

    log( Info ) << "[" << getName( ) << "] Constructed" << endlog( );
} // End of ForceTrigger constructor.

/*
    Configuration hook of the OROCOS component.
*/
bool ForceTrigger::configureHook( )
{
    // Set verified properties.
    if ( msg_contact_ )
    {
        msg_contact_ = false;
    }

    if ( prop_force_constraint_ > 5.0 )
    {
        log( Error ) << "[" << getName( ) << "] Error: force exceeds 5 N. Please validate correct force." << endlog( );
        return false;
    }


    if ( prop_probe_weight_ > 2.0 )
    {
        log( Error ) << "[" << getName( ) << "] Error: probe weight exceeds 2 KG. Please validate correct weight." << endlog( );
        return false;
    }
    //force_constraint_ = prop_force_constraint_;

    log( Info ) << "[" << getName( ) << "] Configured" << endlog( );

    return true;
} // End of configureHook.

/*
    Start hook of the OROCOS component.
*/
bool ForceTrigger::startHook( )
{
    // Set output port and initial msg. 
    msg_contact_ = false ;
    out_contact_.write( msg_contact_ );
    std::fill( msg_net_W_.begin( ), msg_net_W_.end( ), 0 );
    out_net_W_.write( msg_net_W_ );

    // Reset input ports.
    in_W_ati_.clear( ); 
    in_fusion_.clear( ); 

    // Reset flags.
    log( Info ) << "[" << getName( ) << "] Started" << endlog( );

    return true;
} // End of startHook.

/*
    Update hook of the OROCOS component.
*/
void ForceTrigger::updateHook( )
{
    // Read input ports.
    in_W_ati_.read( msg_W_ati_ ); 
    in_fusion_.read( msg_fusion_ ); 

    //  R-P-Y 
    double cy = cos(msg_fusion_ [5] * 0.5);
    double sy = sin(msg_fusion_ [5] * 0.5);
    double cp = cos(msg_fusion_ [4] * 0.5);
    double sp = sin(msg_fusion_ [4] * 0.5);
    double cr = cos(msg_fusion_ [3] * 0.5);
    double sr = sin(msg_fusion_ [3] * 0.5);

    Quaterniond q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;

    // no gravity compensiation;
        msg_net_W_ [0]= msg_W_ati_ [0] ;
        msg_net_W_ [1]= msg_W_ati_ [1] ;
        msg_net_W_ [2]= msg_W_ati_ [2] ;
        msg_net_W_ [3]= msg_W_ati_ [3] ;
        msg_net_W_ [4]= msg_W_ati_ [4] ;
        msg_net_W_ [5]= msg_W_ati_ [5] ;

    if ( (msg_fusion_ [3]) >= 0  && (msg_fusion_ [4]) >=0 &&(msg_fusion_ [5]) >= 0 )
    {
        
        double gx = 2 * (q.x() * q.z() - q.w() * q.y()) * 9.81 * probe_weight_;
        double gy = 2 * (q.w()  * q.x() + q.y() * q.z() )* 9.81 * probe_weight_;
        double gz = -(q.w()* q.w() - q.x()* q.x() - q.y() * q.y() +q.z()* q.z()) * 9.81 *probe_weight_;
        msg_net_W_ [0]= msg_W_ati_ [0] - gx;
        msg_net_W_ [1]= msg_W_ati_ [1] - gy;
        msg_net_W_ [2]= msg_W_ati_ [2] - gz;
        msg_net_W_ [3]= msg_W_ati_ [3] ;
        msg_net_W_ [4]= msg_W_ati_ [4] ;
        msg_net_W_ [5]= msg_W_ati_ [5] ;
    }  


    for ( int i = 0; i < 3; i++  )
    {  
        if ( fabs(msg_net_W_[i])  >= force_constraint_ )
        {
            msg_contact_ = true;
        }
        else
        {
            msg_contact_ = false;
        }
    }


    out_contact_.write( msg_contact_ ); 
    out_net_W_.write( msg_net_W_ );
} // End of updateHook.


/*
    Stop hook of the OROCOS component.
*/
void ForceTrigger::stopHook( )
{
    msg_contact_ = false ;
    out_contact_.write( msg_contact_ ); 
    std::fill( msg_net_W_.begin( ), msg_net_W_.end( ), 0 );
    out_net_W_.write( msg_net_W_ ); 

    log( Info ) << "[" << getName( ) << "] Stopped" << endlog( );
} // End of stopHook.

/*
    Cleanup hook of the OROCOS component.
*/
void ForceTrigger::cleanupHook( )
{
    log( Info ) << "[" << getName( ) << "] Cleaned up" << endlog( );
} // End of cleanupHook.

} // End of namespace forceprobe.

ORO_CREATE_COMPONENT( forcetrigger::ForceTrigger )
 