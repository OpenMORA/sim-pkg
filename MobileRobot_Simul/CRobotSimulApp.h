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

#ifndef CROBOTSIMULAPP_H
#define CROBOTSIMULAPP_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/utils/CRobotSimulator.h>
#include <mrpt/poses/CPose3D.h>

class CRobotSimulApp : public COpenMORAApp
{
public:
    CRobotSimulApp();

    virtual ~CRobotSimulApp();

protected:
    /** called at startup */
    virtual bool OnStartUp();
    /** called when new mail arrives */
    virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
    /** called when work is to be done */
    virtual bool Iterate();
    /** called when app connects to DB */
    virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );

    /** performs the registration for mail */
    bool DoRegistrations();

    /** Restart the simulator */
    bool OnSimulReset();

	// DATA
	double 	m_last_v, m_last_w;

	bool								m_enable_sonar;
	bool								m_enable_laser;
	bool								m_enable_infrared;
	mrpt::gui::CDisplayWindow3DPtr   	m_3dview;
	mrpt::utils::CRobotSimulator		m_robotsim;
	mrpt::maps::CMultiMetricMap			m_map;	
	mrpt::system::TTimeStamp			last_iter;

	unsigned int	m_laser_rays;
	double			m_laser_fov;
	double			m_laser_std_range, m_laser_std_ang;
	double			m_laser_maxrange;
	mrpt::poses::CPose3D  m_laser_pose;
	double			m_sonar_minrange, m_sonar_maxrange;
	double			m_sonar_std_range, m_sonar_std_ang;
	double			m_sonar_aperture;
	std::vector<mrpt::poses::CPose3D> m_sonar_poses;  //!< Sonars to simulate on the robot
	double			m_ir_minrange, m_ir_maxrange;
	double			m_ir_std_range, m_ir_std_ang;
	double			m_ir_aperture;
	std::vector<mrpt::poses::CPose3D> m_ir_poses;  //!< IRs to simulate on the robot

};

#endif
