/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */


/**  @moos_module A simulated mobile robot with a laser scanner, a sonar rig and an infrared sensor ring in a 2D world.
  *  The world is modeled through a grid map.
  */

#include "CRobotSimulApp.h"
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::obs;


CRobotSimulApp::CRobotSimulApp() :
	m_last_v(0), m_last_w(0),
	m_enable_sonar(false),
	m_enable_laser(true),
	m_enable_infrared(false),
	m_enable_rangecamera(false),
	m_robotsim(0,0),
	m_sonar_minrange(0.1), m_sonar_maxrange(5.0),
	m_sonar_std_range(0.1), m_sonar_std_ang(DEG2RAD(2)),
	m_sonar_aperture(DEG2RAD(20)),
	m_ir_minrange(0.06), m_ir_maxrange(0.80),
	m_ir_std_range(0.005), m_ir_std_ang(DEG2RAD(0.5)),
	m_ir_aperture(DEG2RAD(2))
{
	//Default laser initialization
	m_laser.m_rays = 181;
	m_laser.m_fov = M_PI;
	m_laser.m_std_range = 0.01;
	m_laser.m_std_ang = DEG2RAD(0.01);
	m_laser.m_maxrange = 81;
	m_laser.m_pose.setFromValues(0, 0, 0, DEG2RAD(0), 0, 0); //Pitch and roll must always be 0!!

	//Default range camera initialization
	m_rangecam.m_min_range = 0.5f;
	m_rangecam.m_max_range = 5.f;
	m_rangecam.m_fov_v = DEG2RAD(45.f);
	m_rangecam.m_fov_h = DEG2RAD(60.f);
	m_rangecam.m_rows = 15;
	m_rangecam.m_columns = 20;
	m_rangecam.m_std_error = 0.01f;
	m_rangecam.m_pose.setFromValues(0, 0, 1, DEG2RAD(0), DEG2RAD(0), 0); //Roll must be always set to 0!!!
	
	// Default list of sonars to simulate:
	m_sonar_poses.clear();
	m_sonar_poses.push_back( CPose3D(0,0.3,0.3, DEG2RAD(90),DEG2RAD(0),DEG2RAD(0) ) );
	m_sonar_poses.push_back( CPose3D(0.25,0.25,0.3, DEG2RAD(45),DEG2RAD(0),DEG2RAD(0) ) );
	m_sonar_poses.push_back( CPose3D(0.25,-0.25,0.3, DEG2RAD(-45),DEG2RAD(0),DEG2RAD(0) ) );
	m_sonar_poses.push_back( CPose3D(0,-0.3,0.3, DEG2RAD(-90),DEG2RAD(0),DEG2RAD(0) ) );
	m_sonar_poses.push_back( CPose3D(-0.25,0.25,0.3, DEG2RAD(90+45),DEG2RAD(0),DEG2RAD(0) ) );
	m_sonar_poses.push_back( CPose3D(-0.25,-0.25,0.3, DEG2RAD(-90-45),DEG2RAD(0),DEG2RAD(0) ) );

	// Default list of IRs to simulate:
	m_ir_poses.clear();
	m_ir_poses.push_back( CPose3D(0.1, 0.29,0.05, DEG2RAD( 45),DEG2RAD(0),DEG2RAD(0) ) );
	m_ir_poses.push_back( CPose3D(0.1,-0.29,0.05, DEG2RAD(-45),DEG2RAD(0),DEG2RAD(0) ) );
}


bool CRobotSimulApp::OnStartUp()
{
	cout << "CONFIGURING MODULE" << endl;

	EnableCommandMessageFiltering(true);
    DoRegistrations();

    try
    {
		// Load config from .moos mission file:
		//! @moos_param enable_sonar Enable simulating this sensor: "true"/"false"
		m_enable_sonar = m_ini.read_bool("", "enable_sonar",m_enable_sonar);

		//! @moos_param enable_laser Enable simulating this sensor: "true"/"false"
		m_enable_laser = m_ini.read_bool("", "enable_laser",m_enable_laser);

		//! @moos_param enable_infrared Enable simulating this sensor: "true"/"false"
		m_enable_infrared = m_ini.read_bool("", "enable_infrared",m_enable_infrared);

		//! @moos_param enable_rangecamera Enable simulating this sensor: "true"/"false"
		m_enable_rangecamera = m_ini.read_bool("", "enable_rangecamera",m_enable_rangecamera);

		string  sSimplemapFil;
		string  sGridmapFil;

		//! @moos_param simplemap_file The simple map file (.simplemap, .simplemap.gz) to load
		//!        If it's not present, gridmap_image_file will be loaded.
		if (m_MissionReader.GetConfigurationParam("simplemap_file",sSimplemapFil))
		{
			CFileGZInputStream(sSimplemapFil) >> m_map;
			ASSERTMSG_(m_map.m_gridMaps.empty(),"The simplemap file has no XXX");
		}
		else
		//! @moos_param gridmap_image_file The grid map to load, as an image file.
		if (m_MissionReader.GetConfigurationParam("gridmap_image_file",sGridmapFil))
		{
			double grid_res = 0.10;
			double grid_cx = -1;
			double grid_cy = -1;

			m_MissionReader.GetConfigurationParam("gridmap_image_res",grid_res);
			m_MissionReader.GetConfigurationParam("gridmap_image_cx",grid_cx);
			m_MissionReader.GetConfigurationParam("gridmap_image_cy",grid_cy);

			if(!mrpt::system::fileExists(sGridmapFil))
				return MOOSFail("Image file does not exists: %s",sGridmapFil.c_str());

			COccupancyGridMap2DPtr gridmap = COccupancyGridMap2D::Create();
			if (!gridmap->loadFromBitmapFile(sGridmapFil,grid_res,grid_cx,grid_cy))
				return MOOSFail("Error loading bitmap from image: '%s'", sGridmapFil.c_str() );

			//m_map.m_gridMaps.push_back(gridmap);
			m_map.maps.push_back(gridmap);
		}
		else
			return MOOSFail("Neither 'simplemap_file' or 'gridmap_image_file' found in mission file. Quitting.");

		bool show3D = false;
		if (m_MissionReader.GetConfigurationParam("show_3d",show3D) && show3D)
		{
			m_3dview = CDisplayWindow3DPtr( new CDisplayWindow3D("Robot simulator") );
			COpenGLScenePtr scene = m_3dview->get3DSceneAndLock();

			scene->insert( opengl::CGridPlaneXY::Create(-100,100,-100,100,0,1) );

			opengl::CSetOfObjectsPtr map_gl = opengl::CSetOfObjects::Create();
			m_map.getAs3DObject(map_gl);
			scene->insert( map_gl );

			scene->insert( opengl::stock_objects::CornerXYZ() );

			opengl::CSetOfObjectsPtr robot_gl = opengl::stock_objects::RobotPioneer();
			robot_gl->setName("robot");
			scene->insert( robot_gl );

			CPlanarLaserScanPtr laser_gl = CPlanarLaserScan::Create();
			laser_gl->setName("scan");
			laser_gl->setSurfaceColor(0,0,1,0.2);
			robot_gl->insert(laser_gl);  // The laser moves on the robot

			opengl::CPointCloudPtr parts_gl = opengl::CPointCloud::Create();
			parts_gl->setName("particles");
			scene->insert(parts_gl);

			opengl::CPointCloudPtr rangecam_gl = opengl::CPointCloud::Create();
			rangecam_gl->setName("rangecam");
			rangecam_gl->setColor(0,0.8,0.1);
			rangecam_gl->setPointSize(4.f);
			scene->insert(rangecam_gl);

			m_3dview->unlockAccess3DScene();
		}

		m_laser.m_fov = DEG2RAD( m_ini.read_double("","laser_fov_deg",RAD2DEG(m_laser.m_fov)) );

		last_iter = mrpt::system::now();
		OnSimulReset();
		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
	return true;
}

bool CRobotSimulApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pLocalizationPF only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    //MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

	//!  @moos_cmd   RESTART   Reset the simulator state
    if(MOOSStrCmp(sCmd,"RESTART"))
    {
        OnSimulReset();
    }
    return true;
}

// Reset the PF according to the initial distribution:
bool CRobotSimulApp::OnSimulReset ()
{
	try
	{
		cout << "[RobotSimul]: Reseting system" << endl;
		m_robotsim.resetStatus();
		m_last_v = m_last_w = 0.0;
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}

    return true;
}


bool CRobotSimulApp::Iterate()
{
    //double At = GetTimeSinceIterate();
	double At;
	At = mrpt::system::timeDifference( last_iter,mrpt::system::now() );
	last_iter = mrpt::system::now();
	cout << "Iterate time: " << At << endl;
    if (At<0 || At>10.0) At=0;

	// Send the robot the last motion command:	
	m_robotsim.movementCommand(m_last_v,m_last_w);

    // Simulate a time interval:
    m_robotsim.simulateInterval(At);

    // Simulate laser sensors:
    ASSERT_(!m_map.m_gridMaps.empty());

    CPose2D  realPose;
    m_robotsim.getRealPose(realPose);
	CObservation2DRangeScan 	scan;


    if (m_enable_laser)
    {		
		scan.aperture = m_laser.m_fov;
		scan.maxRange = m_laser.m_maxrange;
		scan.sensorPose = m_laser.m_pose;
		scan.sensorLabel = "LASER1";

		m_map.m_gridMaps[0]->laserScanSimulator( scan, realPose, 0.5, m_laser.m_rays, m_laser.m_std_range, 1, m_laser.m_std_ang );

		//!  @moos_var   <SENSOR_LABEL>   The Laser scan "CObservation2DRangeScan" parsed as a std::vector<uint8_t> through ObjectToOctetVector
		mrpt::vector_byte bObs;
		mrpt::obs::CObservation *obs_pointer;
		obs_pointer = &scan;
		mrpt::utils::ObjectToOctetVector(obs_pointer, bObs);
		m_Comms.Notify(scan.sensorLabel, bObs );
		cout << "Laser sent" << endl;
    }

	if (m_enable_rangecamera)
	{
		m_rangecam.CameraScan(*m_map.m_gridMaps[0], realPose);

		// Publish detected poitns as OpenMORA variable
		string sKinect1 = ObjectToString(&m_rangecam.m_points);
		m_Comms.Notify("KINECT1", sKinect1 );
		cout << "Point cloud (from range camera) sent" << endl;

	}

	if (m_enable_sonar)
	{
		// Simulate sonar sensors:
		CObservationRange	sonar;

		sonar.minSensorDistance = m_sonar_minrange;
		sonar.maxSensorDistance = m_sonar_maxrange;
		sonar.sensorConeApperture = m_sonar_aperture;
		sonar.sensorLabel = "SONAR1";
		sonar.sensedData.resize( m_sonar_poses.size() );
		for (size_t i=0;i<m_sonar_poses.size();i++)
		{
			sonar.sensedData[i].sensorID = i;
			sonar.sensedData[i].sensorPose = mrpt::math::TPose3D(m_sonar_poses[i]);
		}
		m_map.m_gridMaps[0]->sonarSimulator(
			sonar, realPose, 0.5,
			m_sonar_std_range, m_sonar_std_ang );

		//!  @moos_publish   SONAR1   Sonar ranges, as a "CObservationRange" passed through "ObjectToString".
		mrpt::vector_byte bObs;
		mrpt::obs::CObservation *obs_pointer;
		obs_pointer = &sonar;
		mrpt::utils::ObjectToOctetVector(obs_pointer, bObs);
		m_Comms.Notify(sonar.sensorLabel, bObs );
		cout << "Sonar sent" << endl;
	}

	if (m_enable_infrared)
	{
		// Simulate infrared sensors:
		CObservationRange	ir;

		ir.minSensorDistance = m_ir_minrange;
		ir.maxSensorDistance = m_ir_maxrange;
		ir.sensorConeApperture = m_ir_aperture;
		ir.sensorLabel = "INFRARED1";
		ir.sensedData.resize( m_ir_poses.size() );
		for (size_t i=0;i<m_ir_poses.size();i++)
		{
			ir.sensedData[i].sensorID = i;
			ir.sensedData[i].sensorPose = mrpt::math::TPose3D(m_ir_poses[i]);
		}
		m_map.m_gridMaps[0]->sonarSimulator(
			ir, realPose, 0.5,
			m_ir_std_range, m_ir_std_ang );

		//!  @moos_publish   INFRARED1   Infrared ranges, as a "CObservationRange" passed through "ObjectToString".
		mrpt::vector_byte bObs;
		mrpt::obs::CObservation *obs_pointer;
		obs_pointer = &ir;
		mrpt::utils::ObjectToOctetVector(obs_pointer, bObs);
		m_Comms.Notify(ir.sensorLabel, bObs );
		cout << "IR sent" << endl;
	}


	// Publish new robot pose:
	CPose2D  odo;
	m_robotsim.getOdometry(odo);

	//!  @moos_publish   ODOMETRY   The robot absolute odometry in format "[x y phi]"
	string sOdo;
	odo.asString(sOdo);
    m_Comms.Notify("ODOMETRY", sOdo );	

	// Publish complete odometry as CObservation:
	mrpt::obs::CObservationOdometryPtr odom = mrpt::obs::CObservationOdometry::Create();
	odom->odometry = odo;
	odom->timestamp = mrpt::system::now();
	odom->hasVelocities = false;
	odom->velocityLin = 0.0;
	odom->velocityAng = 0.0;
	odom->hasEncodersInfo = false;
	odom->encoderLeftTicks = 0;
	odom->encoderRightTicks = 0;

	mrpt::vector_byte vec_odom;
	mrpt::utils::ObjectToOctetVector(odom.pointer(), vec_odom);
	//!  @moos_publish  ODOMETRY_OBS The robot absolute odometry as mrpt::obs::CObservationOdometry
	m_Comms.Notify("ODOMETRY_OBS", vec_odom);
	

	// Update 3D view:
	if (m_3dview.present())
	{
		// If present, parse PF particles just to visualize them:
		CMOOSVariable *varLocParts = GetMOOSVar("LOCALIZATION_PARTICLES");
		mrpt::math::CMatrixDouble parts_display;
		if (varLocParts && varLocParts->IsFresh())
		{
			varLocParts->SetFresh(false);
			parts_display.fromMatlabStringFormat( varLocParts->GetStringVal());
		}

		COpenGLScenePtr scene = m_3dview->get3DSceneAndLock();
		CRenderizablePtr  gl_robot = scene-> getByName("robot");
		if (gl_robot)
			gl_robot->setPose( realPose );

		if (m_enable_laser)
		{
			CRenderizablePtr  gl_scan = scene->getByName("scan");
			if (gl_scan)
			{
				opengl::CPlanarLaserScanPtr  laser_gl = opengl::CPlanarLaserScanPtr(gl_scan);
				laser_gl->setScan( scan );
			}
		}

		// Particles:
		if (size(parts_display,1)>0 && size(parts_display,2)==3)
		{
			CRenderizablePtr  gl_parts = scene->getByName("particles");
			if (gl_parts)
			{
				opengl::CPointCloudPtr gl_particles = opengl::CPointCloudPtr(gl_parts);
				gl_particles->setPose( CPose3D(0,0,0.04,0,0,0) );
				gl_particles->setPointSize(3.0);
				gl_particles->clear();
				gl_particles->setColor(0,0,1);

				const size_t N =size(parts_display,1);
				for(size_t i=0;i<N;++i)
					gl_particles->insertPoint( parts_display.get_unsafe(i,0),parts_display.get_unsafe(i,1),0);
			}
		}

		//Point cloud from range camera
		if (m_enable_rangecamera)
		{
			CRenderizablePtr  gl_rangecam = scene->getByName("rangecam");
			if (gl_rangecam)
			{
				opengl::CPointCloudPtr  rangecam_gl = opengl::CPointCloudPtr(gl_rangecam);
				std::vector<float> xp, yp, zp;
				m_rangecam.m_points.getAllPoints(xp, yp, zp);
				rangecam_gl->setAllPoints(xp, yp, zp);
				rangecam_gl->setPose(realPose);
			}
		}

		m_3dview->unlockAccess3DScene();
		m_3dview->repaint();
	}

    return true;
}



bool CRobotSimulApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CRobotSimulApp::DoRegistrations()
{
    AddMOOSVariable( "LOCALIZATION_PARTICLES",  "LOCALIZATION_PARTICLES","LOCALIZATION_PARTICLES", 5.0 /* Maximum period to be informed */ );

    this->m_Comms.Register("MOTION_CMD_V",0);
    this->m_Comms.Register("MOTION_CMD_W",0);
	
	//! @moos_subscribe SHUTDOWN
	this->m_Comms.Register("SHUTDOWN", 0 );

    RegisterMOOSVariables();
    return true;
}


bool CRobotSimulApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	//!  @moos_subscribe  MOTION_CMD_V, MOTION_CMD_W
	for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
	    const CMOOSMsg &m = *it;

	    if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_V"))
		{
            m_last_v = m.GetDouble();
			//cout << "New v= " << m_last_v << endl;
		}

	    if (MOOSStrCmp(m.GetKey(),"MOTION_CMD_W"))
		{
            m_last_w = m.GetDouble();
			//cout << "New w= " << m_last_w << endl;
		}
		
		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
		{			
			this->RequestQuit();
			
		}
	}
    UpdateMOOSVariables(NewMail);
    return true;
}

