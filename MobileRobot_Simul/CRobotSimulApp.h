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
#include <mrpt/math.h>


class CSimRangeCamera {
public:
	mrpt::maps::CSimplePointsMap	m_points;
	mrpt::poses::CPose3D			m_pose;

	float				m_min_range;
	float				m_max_range;
	float				m_fov_v;
	float				m_fov_h;
	unsigned int		m_rows;
	unsigned int		m_columns;
	float				m_std_error;

	void CorrectFloorPoints()
	{
		using namespace mrpt::math;
		TSegment3D ray;
		TPoint3D p1,p2,pint(0,0,0);
		TObject3D pintobj;
		TPlane ground(0,0,1,0);
		std::vector <float> x, y, z;

		m_points.getAllPoints(x,y,z,1);
		p2.x = m_pose[0];
		p2.y = m_pose[1];
		p2.z = m_pose[2];
		ray.point2 = p2;

		for (unsigned int i=0; i<m_points.size();i++)
		{
			if (z[i] < 0)
			{
				p1.x = x[i]; p1.y = y[i]; p1.z = z[i];
				ray.point1 = p1;
				mrpt::math::intersect(ray,ground,pintobj);
				ASSERT_(pintobj.isPoint())
				pintobj.getPoint(pint);
				x[i] = pint.x; y[i] = pint.y; z[i] = pint.z;
			}
		}
		m_points.setAllPoints(x,y,z);
	}

	void CorrectCeiling(float height)
	{
		using namespace mrpt::math;
		TSegment3D ray;
		TPoint3D p1,p2,pint(0,0,0);
		TObject3D pintobj;
		TPlane ceiling(0,0,1,-height);
		std::vector <float> x, y, z;

		m_points.getAllPoints(x,y,z,1);

		p2.x = m_pose[0];
		p2.y = m_pose[1];
		p2.z = m_pose[2];
		ray.point2 = p2;

		for (unsigned int i=0; i<m_points.size();i++)
		{
			if (z[i] > height)
			{
				p1.x = x[i]; p1.y = y[i]; p1.z = z[i];
				ray.point1 = p1;
				intersect(ray,ceiling,pintobj);
				ASSERT_(pintobj.isPoint())
				pintobj.getPoint(pint);
				x[i] = pint.x; 	y[i] = pint.y; z[i] = pint.z;
			}
		}
		m_points.setAllPoints(x,y,z);
	}

	void CorrectRanges()
	{
		using namespace std;
		std::vector <float> x, y, z;
		std::vector <bool> deletion;

		m_points.getAllPoints(x,y,z,1);

		for (unsigned int i=0; i<m_points.size();i++)
		{
			if ((m_pose.distance3DTo(x[i],y[i],z[i]) < m_min_range)||(m_pose.distance3DTo(x[i],y[i],z[i]) > m_max_range))
				deletion.push_back(1);

			else
				deletion.push_back(0);

		}
		m_points.applyDeletionMask(deletion);
	}

	void CameraScan(const mrpt::maps::COccupancyGridMap2D &map, const mrpt::poses::CPose3D &robotpose)
	{
		unsigned int acc_factor = std::max(1,mrpt::utils::round<double>(80.f/m_columns));
		float incrz;
		mrpt::obs::CObservation2DRangeScan m_auxlaser;
		mrpt::poses::CPose2D scanpose2d;
		mrpt::math::TPoint3D point;
		mrpt::maps::CSimplePointsMap row_points;
		row_points.insertionOptions.minDistBetweenLaserPoints = 0;
		m_points.clear();

		scanpose2d.x(robotpose[0]);
		scanpose2d.y(robotpose[1]);
		scanpose2d.phi(robotpose[3]);
		mrpt::poses::CPose3D auxpose = m_pose;
		auxpose.setYawPitchRoll(m_pose[3], 0.f, 0.f);
		m_auxlaser.setSensorPose(m_pose);
		m_auxlaser.aperture = m_fov_h;

		//acc_factor is used to get a higher resolution
		map.laserScanSimulator( m_auxlaser, scanpose2d, 0.5f, acc_factor*m_columns, m_std_error, 1, 0);
		row_points.insertObservation(&m_auxlaser);

		for (unsigned int i=0;i<m_rows;i++)
			for (unsigned int j=0;j<m_columns;j++)
			{
				if (row_points.size() > acc_factor*j)
				{
					row_points.getPoint(acc_factor*j,point.x,point.y,point.z);
					incrz = m_pose.distance3DTo(point.x,point.y,point.z)*tan((float(i)/(m_rows-1)-0.5)*m_fov_v-m_pose[4])*cos((float(j)/(m_columns-1)-0.5)*m_fov_h);
					point.z = point.z + incrz;

					m_points.insertPoint(point);
				}
			}

		row_points.clear();

		CorrectFloorPoints();
		CorrectCeiling(2.5f);  //Default: ceiling height = 3 meters
		CorrectRanges();
	}
};

class CSimLaser {
public:
	unsigned int	m_rays;
	double			m_fov;
	double			m_std_range, m_std_ang;
	double			m_maxrange;
	mrpt::poses::CPose3D  m_pose;
};



class CRobotSimulApp : public COpenMORAApp
{
public:
    CRobotSimulApp();

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
	bool								m_enable_rangecamera;
	mrpt::gui::CDisplayWindow3DPtr   	m_3dview;
	mrpt::utils::CRobotSimulator		m_robotsim;
	mrpt::maps::CMultiMetricMap			m_map;	
	mrpt::system::TTimeStamp			last_iter;
	CSimLaser							m_laser;
	CSimRangeCamera						m_rangecam;


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
