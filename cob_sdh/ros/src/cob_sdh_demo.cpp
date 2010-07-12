//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!

    \file
     \section sdhlibrary_cpp_sdh_demo_contact_grasping_general General file information

       \author   Dirk Osswald
       \date     2009-08-02

     \brief
       Simple script to do grasping using tactile sensor info feedback.
       See \ref demo_contact_grasping__help__ "__help__" and online help ("-h" or "--help") for available options.



     \section sdhlibrary_cpp_sdh_demo_contact_grasping_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_contact_grasping_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2009-12-04 17:05:53 +0100 (Fr, 04 Dez 2009) $
         \par SVN file revision:
           $Id: demo-contact-grasping.cpp 5022 2009-12-04 16:05:53Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_contact_grasping_changelog Changelog of this file:
         \include demo-contact-grasping.cpp.log

*/
/*!
  @}
*/
//======================================================================

#include <iostream>
#include <vector>

//Ros include
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Include the cSDH interface
#include "cob_sdh/sdh.h"
#include "cob_sdh/util.h"
#include "cob_sdh/sdhlibrary_settings.h"
#include "cob_sdh/basisdef.h"
#include "cob_sdh/dsa.h"
//#include "cob_sdhoptions.h"
#include "cob_sdh/dsaboost.h"
#include "cob_sdh/dbg.h"

//include msgs
#include <cob_msgs/JointCommand.h>
#include <cob_msgs/TactileForce.h>
#include <std_msgs/Float32MultiArray.h>
#include <cob_actions/JointCommandAction.h>
#include <cob_msgs/ContactArea.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/demoinfo.h>
#include <cob_srvs/GetFingerXYZ.h>
#include <cob_srvs/Force.h>
#include <cob_srvs/SetOperationMode.h>


USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_demo_contact_grasping_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_contact_grasping__help__
char const* __help__      =
    "Simple script to do grasping with tactile sensor info feedback:\n"
    "The hand will move to a pregrasp pose (open hand). You can then\n"
    "reach an object to grasp into the hand. The actual grasping\n"
    "is started as soon as a contact is detected. The finger\n"
    "joints then try to move inwards until a certain\n"
    "force is reached on the corresponding tactile sensors.\n"
    "\n"
    "- Example usage:\n"
    "  - Make SDH connected to port 2 = COM3 with tactile sensors:\n"
    "    connected to port 3 = COM4 grasp:\n"
    "    > demo-contact-grasping -p 2  --dsaport=3\n"
    "     \n"
    "  - Make SDH connected to USB to RS232 converter 0 and with tactile sensors:\n"
    "    connected to USB to RS232 converter 1 grasp:\n"
    "    > demo-contact-grasping --sdh_rs_device=/dev/ttyUSB0  --dsa_rs_device=/dev/ttyUSB0\n"
    "     \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected to:\n"
    "    - port 2 = COM3 (joint controllers) and \n"
    "    - port 3 = COM4 (tactile sensor controller) \n"
    "    > demo-contact-grasping --port=2 --dsaport=3 -v\n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-contact-grasping.cpp 5022 2009-12-04 16:05:53Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_contact_grasping_cpp_vars
//  @}

char const* usage =
  "usage: demo-contact-grasping [options]\n" ;








                             
 

//-----------------------------------------------------------------
/*void ForceCallback(const cob_msgs::TactileForceConstPtr& msg)
   { int i;
     
       
       for (i=0; i<(int)(sizeof(msg->data)/sizeof(double));i++)
         { 
           force.push_back (msg->data[i]);
         }
      
       
         
     
   } 
*/

double area2;
double area3;



void AreaCallback(const cob_msgs::ContactAreaConstPtr& msg)
    { 
       area2 = msg->data2;
       area3 = msg->data3;
      
    }
    

 
std::vector<double> mv_;

void Callback(const std_msgs::Float32MultiArrayConstPtr& msg)
    {
                   
       for (int i=0; i< (int)(sizeof(msg->data)/sizeof(float));i++)   
        {
         mv_.push_back (msg->data[i]);
        }
          
    } 
     
    
void GotoStartPose(std::vector<double>& fa)
{
    printf("\nPreparing for grasping, opening hand (using POSE controller)...\n");
    //std::vector<double> fa;
    fa.resize(0);
    fa.push_back( -1.57 );
    fa.push_back( -1.57 ); //joint_palm_finger11
    fa.push_back( 0.7854 ); //joint_finger11_finger12
    fa.push_back( -0.7854 );
    fa.push_back( 0 );
    fa.push_back( 0 );
    fa.push_back( 0 );
    fa.push_back( 0 );
    fa.push_back( 0 );
    
    		

}
//-----------------------------------------------------------------

void AxisAnglesToFingerAngles( std::vector<double> aa, std::vector<double>& fa0, std::vector<double>& fa1, std::vector<double>& fa2 )
{
   fa0[0] = aa[0];
   fa0[1] = aa[1];
   fa0[2] = aa[2];

   fa1[0] = 0.0;
   fa1[1] = aa[3];
   fa1[2] = aa[4];

   fa2[0] = aa[0];
   fa2[1] = aa[5];
   fa2[2] = aa[6];
}
//-----------------------------------------------------------------


int main( int argc, char** argv )
{               
SDH_ASSERT_TYPESIZES();



// initialize ROS, spezify name of node
ros::init(argc, argv,"cob_sdh_demo");
//Node handle
ros::NodeHandle nh_;
//Topic to publish
actionlib::SimpleActionClient<cob_actions::JointCommandAction> ac_("JointCommand",true);
//Goal to send
cob_actions::JointCommandGoal goal;
//command to send
cob_msgs::JointCommand command_;



    
//Services to call                
ros::ServiceClient srvClient_SetAxisTargetAcceleration_ = nh_.serviceClient<cob_srvs::Trigger>("SetAxisTargetAcceleration");
ros::ServiceClient srvClient_DemoInfo_ = nh_.serviceClient<cob_srvs::demoinfo>("DemoInfo");
ros::ServiceClient srvClient_GetFingerXYZ_ = nh_.serviceClient<cob_srvs::GetFingerXYZ>("GetFingerXYZ");
ros::ServiceClient srvClient_Updater_ =  nh_.serviceClient<cob_srvs::Trigger>("DSAUpdater");   
ros::ServiceClient srvClient_CloseHand_ =  nh_.serviceClient<cob_srvs::Trigger>("CloseHand");
ros::ServiceClient srvClient_Init = nh_.serviceClient<cob_srvs::Trigger>("Init");  
ros::ServiceClient srvClient_Force_ = nh_.serviceClient<cob_srvs::Force>("Force");	
ros::ServiceClient srvClient_SetOperationMode = nh_.serviceClient<cob_srvs::SetOperationMode>("SetOperationMode");	
		std::vector<std::string> JointNames_;
		std::vector<std::string> JointNamesAll_;
		std::vector<int> axes_;
		std::vector<double> targetAngles_; // in degrees
		int debug_level=0;

                
std::vector<SDH::cSDH::eAxisState> state_;

double timeout_;


                	
 cDBG cdbg( false, "red" ); 
                               

//SDH_ASSERT_TYPESIZES();
    
	


 

 
    // initialize debug message printing:
    
    cdbg.SetFlag( debug_level > 0 );
    //cdbg.SetOutput( options.debuglog );

    //g_sdh_debug_log = options.debuglog;

    cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

    // reduce debug level for subsystems
    debug_level-=1;
    //---------------------
    // external code
        bool srv_querry = false;
	    int srv_execute = 1;
	    std::string srv_errorMessage = "no error";
                
    std::cout << "initiating the hand";            
                
    cob_srvs::Trigger srv0;
    srv_querry = srvClient_Init.call(srv0);
    srv_execute = srv0.response.success;
    srv_errorMessage = srv0.response.errorMessage.data.c_str();
     
    try
    {
           

        timeout_ = 1.0; // a real timeout is needed to make the closing of the connections work in case of errors / interrupt

         /*printf("Connecting to tactile sensor controller. This may take up to 8 seconds...");
        cDSA ts = cDSA(0, dsadevicenum_, dsadevicestring_.c_str());
        printf("OK\n");
        */
        
        //###
        // Prepare for grasping: open hand:
        std::vector<double> pose;
        pose.resize(9);
    
        GotoStartPose(pose);
	std::cerr << "Pose generated " << pose[0] << " " << pose[1] << " " << pose[2] << "\n";
        command_.positions.resize(9); 
	for (int i = 0; i<9; i++ )
	  {           		             
	    command_.positions[i] = pose[i]; 
            
	  }
	goal.command = command_;
	goal.hasvelocity = false; 
	ac_.sendGoal(goal);
        
         
        //###


        // Start reading tactile sensor info in a thread:
        //cDSAUpdater dsa_updater( &ts, 8 );
        //boost::this_thread::sleep(boost::posix_time::milliseconds(200)); // give the updater a chance to read in first frame
                
        //cDSA::sContactInfo contact_info;

        //###
#if 0
        // for debugging, just output the local preprocessing results:
        cIsGraspedByArea is_grasped_obj( &ts );
        double expected_area_ratio[6] = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
        for ( int i=0; i+unused_opt_ind<argc; i++ )
        {
            int rc = sscanf( argv[unused_opt_ind+i], "%lf", &expected_area_ratio[i] );
            assert( rc == 1 );
        }
        is_grasped_obj.SetCondition( expected_area_ratio );

        while  ( true )
        {
            for ( int m=0; m < 6; m++ )
            {
                contact_info = ts.GetContactInfo( m );
                printf( "  m=%d age=%ldms force=%6.1f x=%6.1f y=%6.1f area=%6.1f\n",
                        m,
                        ts.GetAgeOfFrame( (cDSA::sTactileSensorFrame*) &ts.GetFrame() ),
                        contact_info.force, contact_info.cog_x, contact_info.cog_y, contact_info.area );
            }
            printf( "=> IsGrasped: %d\n", is_grasped_obj.IsGrasped() );

            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
#endif
        //###

        //###
        // wait for contact to start grasping
        // double contact_area;  
                    
      
         
        printf("\nPress any tactile sensor on the SDH to start searching for contact...") ; fflush( stdout );
        mv_.resize(6);
        int is_grasped_no = 0;   
        float threshold = 10; 
        
        ros::Subscriber sub = nh_.subscribe("/tactile_tools/mean_values", 100, Callback);
 
       
      
      cDBG cdbg(false,"red");
         do
        {
            //contact_area = 0.0;
            //		mv = ros.getMsg("/tactile_tools/mean_values");
          
            is_grasped_no = 0;   
            
            for ( int m=0; m <(int) (sizeof(mv_)/sizeof(double)); m++ )
            {
            //    contact_area += ts.GetContactArea( m );
                              
                     		if( mv_[m] > threshold)
            			       is_grasped_no += 1;
                //printf( "  m=%d area2=%6.1f\n", m, ts.GetContactArea( m ) );
            }

            printf("  contact area too small\n");
                                    
            ros::Subscriber sub = nh_.subscribe("/tactile_tools/mean_values", 100, Callback);
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
            
        //} while ( contact_area < desired_start_contact_area );
        } while ( is_grasped_no <2 );
      
      
        
         
       
       
       
        // wait until that contact is released on the middle finger
        ros::Subscriber sub1 = nh_.subscribe("area_data", 100, AreaCallback);  
        
        double desired_start_contact_area = 5.0;
        while ( (area2 + area3) > desired_start_contact_area )
        {
           ros::Subscriber sub1 = nh_.subscribe("area_data", 100, AreaCallback);
        
           boost::this_thread::sleep(boost::posix_time::milliseconds(200)); // give the updater a chance to read in first frame       
        }


        printf("  OK, contact detected: \n") ; fflush( stdout );
        //###

        //###
        // start grasping
        double desired_force = 5.0; //options.desired_force // the desired force that every sensor patch should reach
        double vel_per_force = 2.5;     // factor for converting force to velocity
        double loop_time = 0.01;
        int loop_time_ms = (int) (loop_time * 1000.0);

        bool finished = false;
        printf( "Simple force based grasping starts:\n" );
        printf( "  Joints of the opposing fingers 1 and 3 (axes 1,2,5,6) will move in\n" );
        printf( "  positive direction (inwards) as long as the contact force measured\n" );
        printf( "  by the tactile sensor patch of that joint is less than\n" );
        printf( "  the desired force %f\n", desired_force );
        printf( "  This will use the velocity with acceleration ramp controller.\n" );
        printf( "\n" );
        printf( "  Press a tactile sensor on the middle finger 2 to stop!\n" );
        fflush( stdout );
          

       

        // switch to velocity with acceleration ramp controller type:
       /* hand.SetController( hand.eCT_VELOCITY_ACCELERATION );
        hand.SetAxisTargetAcceleration( hand.All, 100 );
        cdbg << "max=" << hand.GetAxisMaxVelocity( hand.all_real_axes ) << "\n";*/
                cob_srvs::Trigger srv1;
                srv_querry = srvClient_SetAxisTargetAcceleration_.call(srv1);
                srv_execute = srv1.response.success;
                srv_errorMessage = srv1.response.errorMessage.data.c_str();
          
        

                
               

        std::vector<double> aaa; aaa.resize(9); // axis actual angles
        std::vector<double> ata; ata.resize(9); // axis target angles
        std::vector<double> atv; atv.resize(9); // axist target velocities
        std::vector<double> fta0(3); // finger target angles
        std::vector<double> fta1(3); // finger target angles
        std::vector<double> fta2(3); // finger target angles
        std::vector<double> fta[3];
        fta[0] = fta0;
        fta[1] = fta1;
        fta[2] = fta2;

        std::vector<double> min_angles ;
        std::vector<double> max_angles ;
        std::vector<double> max_velocities ;
         
        //Call service to get infomation for following variables      
                cob_srvs::demoinfo srv2;
        
                srv_querry = srvClient_DemoInfo_.call(srv2);
                srv_execute = srv2.response.success;
                srv_errorMessage = srv2.response.errorMessage.data.c_str(); 
       
        for (int i=0; i<7; i++)
          {
                 
                    
                    min_angles.push_back((srv2.response.MinAngle)[i]);
                    
                    max_angles.push_back( (srv2.response.MaxAngle)[i]);
                    
                    max_velocities.push_back ((srv2.response.MaxVelocity)[i]);
                    
                    atv.push_back ((srv2.response.TargetVelocity)[i]);
                    
           }
                      
        
        while (true) //!finished:
        {
            int nb_ok = 0;

            //###
            // check for stop condition (contact on middle finger)
            cDSA::sContactInfo contact_middle_prox; contact_middle_prox.area =srv2.response.Contact_info2_area;
            cDSA::sContactInfo contact_middle_dist; contact_middle_dist.area =srv2.response.Contact_info3_area;
            if ( contact_middle_prox.area + contact_middle_dist.area > desired_start_contact_area )
            {
                printf ("\nContact on middle finger detected! Stopping demonstration.\n") ; fflush( stdout );
                finished = true;
                break;
            }
            //
            //###


            // Get current position of axes:
            // (Limited to the allowed range. This is necessary since near the
            //  range limits an axis might report an angle slightly off range.)
                cob_srvs::demoinfo srv2;
                srv_querry = srvClient_DemoInfo_.call(srv2);
                srv_execute = srv2.response.success;
                srv_errorMessage = srv2.response.errorMessage.data.c_str();
            
            for (int i=0; i<7; i++)
          {
            
             aaa.push_back ((srv2.response.ActualAngle)[i]);
          }  
            
            
            ToRange( aaa, min_angles, max_angles );
            ata = aaa;
            //std::vector<double> force ;
            
                    
            
            

             //for the selected axes:
            int ais[] = { 1,2, 5,6 }; // indices of used motor axes
            int mis[] = { 0,1, 4,5 }; // indices of used tactile sensors
            
            std::vector<double> force; force.resize(4);
                        
            cob_srvs::Force srv5;
            srv_querry = srvClient_Force_.call(srv5);
            
            for (int i =0; i < (int)(sizeof(srv5.response.force_data)/sizeof(double)); i++)
               { 
                force.push_back ((srv5.response.force_data)[i]) ;
                }           
            
            srv_execute = srv5.response.success;
            srv_errorMessage = srv5.response.errorMessage.data.c_str();
            
            
            for ( int i=0; i < (int) (sizeof( ais ) / sizeof( int )); i++ )
            {
                int ai = ais[i];
                int mi = mis[i];
                // read the contact force of the tactile sensor of the axis
                //contact_info  = ts.GetContactInfo( mi );
                //cdbg << "%d,%d,%d  force=%6.1f x=%6.1f y=%6.1f area=%6.1f\n" % (ai, fi, part, force, cog_x, cog_y, area) # pylint: disable-msg=W0104
           
             if((int)force.size() >= mi)
	            
	           {    cdbg << mi << " force=" << force[mi] << "\n";
	            
                   // translate the measured force into a new velocity for the axis
                   // in order to reach the desired_force
                
                
                    atv[ai] = (desired_force - force[mi]) * vel_per_force;
                
	            	
                
                // limit the calculated target velocities to the allowed range:
                atv[ai] = ToRange( atv[ai], -max_velocities[ai], max_velocities[ai] );

                if ( force[mi] < desired_force )
                {
                    cdbg << "closing axis " << ai << " with " << atv[ai] << " deg/s\n";
                }
                else if ( force[mi] > desired_force )
                {
                    cdbg << "opening axis " << ai << " with " << atv[ai] << " deg/s\n";
                }
                else
                {
                    cdbg << "axis " << ai << " ok\n";
                    nb_ok += 1;

                }
              }
              else 
              cdbg << "mi is greater then force size\n";  

                //###
                // check if the fingers would get closer than 10mm with the calculated position:
                // calculate future position roughly according to determined velocity:
                ata[ai] += atv[ai] * loop_time; // s = v * t  =>  s(t') = s(t) + v * dt


                AxisAnglesToFingerAngles( ata, fta[0], fta[1], fta[2] );

                // TODO: CheckFingerCollisions not implemented in SDHLibrary-C++
#if 0
                (cxy, (c01,d01), (c02,d02), (c12,d12)) = hand.CheckFingerCollisions( fta[0], fta[1], fta[2] );
                d_min = min( d01, d02, d12);
                if ( (cxy || d_min < 2.0)  && force < desired_force )
                {
                    // if there would be a collision then do not move the current axis there
                    printf( "not moving axis %d further due to internal collision (d_min=%f)\n", ai, d_min );
                    ata[ai] = aaa[ai];
                    atv[ai] = 0.0;
                }
#else
                // simple approach for now: dont let any finger move into the other fingers half
                int fi = ( (ai<=2) ? 0 : 2 );
                //std::vector<double> xyz = hand.GetFingerXYZ( fi, fta[fi] );
                

                std::vector<double> xyz;
                cob_srvs::GetFingerXYZ srv3;
                srv3.request.number = fi;
                for (i=0; i<(int)(sizeof(fta[fi])/sizeof(double));i++)
                  {
                     srv3.request.value[i] = fta[fi][i];
                   }
                
               srv_querry = srvClient_GetFingerXYZ_.call(srv3);
                
                 for (i=0; i<3;i++)
                  { 
                    
                    xyz.push_back(srv3.response.data[i]);
                  }
          
                    srv_execute = srv3.response.success;
                    srv_errorMessage = srv3.response.errorMessage.data.c_str(); 

                switch (fi)
                {
                case 0:
                    if ( xyz[0] <= 6.0 )
                    {
                        // if there would be a collision then do not move the current axis there
                        printf( "not moving axis %d further due to quadrant crossing of finger %d xyz= %6.1f,%6.1f,%6.1f\n", ai, fi, xyz[0],xyz[1],xyz[2] );
                        ata[ai] = aaa[ai];
                        atv[ai] = 0.0;
                    }
                    break;
                case 2:
                    if ( xyz[0] >= -6.0 )
                    {
                        // if there would be a collision then do not move the current axis there
                        printf( "not moving axis %d further due to quadrant crossing of finger %d xyz= %6.1f,%6.1f,%6.1f\n", ai, fi, xyz[0],xyz[1],xyz[2] );
                        ata[ai] = aaa[ai];
                        atv[ai] = 0.0;
                    }
                    break;
                default:
                    assert( fi == 0  || fi == 2 );
                }
#endif
                //
                //###

            } 
            // a new target velocity has been calculated from the tactile sensor data, so move accordingly
            cdbg.PDM( "moving with %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f deg/s\n", atv[0], atv[1], atv[2], atv[3], atv[4], atv[5], atv[6] );
            
            cob_srvs::SetOperationMode srv6;
            srv6.request.operationMode.data = "velocity";
            //command_.velocities.resize(command_param[command_nr].size());
            ROS_INFO("changing operation mode to: %s controll", srv6.request.operationMode.data.c_str());
            srv_querry = srvClient_SetOperationMode.call(srv6);
            srv_execute = srv6.response.success;
            srv_errorMessage = srv6.response.errorMessage.data.c_str();
            
            command_.velocities.resize(9); command_.positions.resize(9);
		    
		    command_.velocities[3] = atv[0];
			command_.velocities[7] = atv[1];
			command_.velocities[8] = atv[2]; 			
			command_.velocities[1] = atv[3];
			command_.velocities[2] = atv[4]; 
			command_.velocities[4] = atv[5];
			command_.velocities[5] = atv[6];
			command_.velocities[0] = 0;
			command_.velocities[6] = 0;
			 
			
					    
		    goal.command = command_;
		    goal.hasvelocity = true; 
		    ac_.sendGoal(goal);
           
            finished = (nb_ok == 6);

            fflush( stdout );
            boost::this_thread::sleep(boost::posix_time::milliseconds(loop_time_ms));
        
        }
        cdbg << "after endless loop\n";



        //###
        // open up again
        //GotoStartPose(pose); 
            cob_srvs::SetOperationMode srv7;
            srv7.request.operationMode.data = "position";
            //command_.velocities.resize(command_param[command_nr].size());
            ROS_INFO("changing operation mode to: %s controll", srv7.request.operationMode.data.c_str());
            srv_querry = srvClient_SetOperationMode.call(srv7);
            srv_execute = srv7.response.success;
            srv_errorMessage = srv7.response.errorMessage.data.c_str();
          
        command_.positions.resize(9); 
		for (int i = 0; i<9; i++ )
		{
                  	command_.positions[i] = (double)pose[i]; 
		}
		goal.command = command_;
		goal.hasvelocity = false; 
		ac_.sendGoal(goal);
       
        //
        //###


        //###
        // stop sensor:
        //dsa_updater.interrupt();

        //ts.Close();
        //cdbg << "Successfully disabled tactile sensor controller of SDH and closed connection\n";

        // Close the connection to the SDH and DSA
        //hand.Close();
        //cdbg << "Successfully disabled joint controllers of SDH and closed connection\n";
                cob_srvs::Trigger srv4;
                srv_querry = srvClient_CloseHand_.call(srv4);
                srv_execute = srv4.response.success;
                srv_errorMessage = srv4.response.errorMessage.data.c_str();  

          


    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-contact-grasping main(): An exception was caught: " << e->what() << "\n";
        delete e;
    }
}
//----------------------------------------------------------------------


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored)
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================]
