/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering   
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_light
 * Description: Switch robots led color by sending data to
 * the led-µC over serial connection.
 *                              
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *          
 * Author: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 * Supervised by: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 *
 * Date of creation: August 2012
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <spektrumviz.h>

SpektrumViz::SpektrumViz(mybeat::BeatController* beatController)
{
	_beatController = beatController;
#if (GTKMM_MAJOR_VERSION == 2 && GTKMM_MINOR_VERSION >= 4)
	signal_expose_event().connect(sigc::mem_fun(*this, &SpektrumViz::on_expose_event), false);
#elif (GTKMM_MAJOR_VERSION == 3)
	signal_draw().connect(sigc::mem_fun(*this, &SpektrumViz::on_draw), false);
#endif
	_beatController->signalProcessingDone()->connect(boost::bind(&SpektrumViz::soundProcessingDone,this));
	m_fps.start();
}
SpektrumViz::~SpektrumViz(){;}


#if (GTKMM_MAJOR_VERSION == 2 && GTKMM_MINOR_VERSION >= 4)
bool SpektrumViz::on_expose_event(GdkEventExpose* event)
{
	Glib::RefPtr<Gdk::Window> window = get_window();
	Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
#elif (GTKMM_MAJOR_VERSION == 3)
bool SpektrumViz::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
#endif
	static int fps_counter = 0;
	static double time_elapsed = 0;
	static double fps_time = 0;
		Gtk::Allocation allocation = get_allocation();
	const int width = allocation.get_width();
	const int height = allocation.get_height();

	// scale to unit square and translate (0, 0) to be (0.5, 0.5), i.e.
	// the center of the window
	
	//left side
	for(uint16_t idx=0; idx < _beatController->getAnalysers().size(); idx++)
	{
		cr->set_line_width(2);
		cr->set_source_rgba(0.423, 0.482, 0.545, 0.8);   // gray

		// for(uint16_t i=0;i<DEFAULT_RECORD_SIZE/2;i++)
		// {
		// 	//Draw the function itself
		// 	cr->move_to( ((double)i/(DEFAULT_RECORD_SIZE/DEFAULT_CHANNELS-1.0)*(double)width/2.0) + (((double)width/2.0)*idx), (double)height );
		// 	cr->line_to( ((double)i/(DEFAULT_RECORD_SIZE/DEFAULT_CHANNELS-1.0)*(double)width/2.0) + (((double)width/2.0)*idx), (double)height- (_beatController->getFFTs().at(idx)->get_magnitude(i)/_beatController->getFFTs().at(idx)->get_magnitude_max())*(double)height);
		// 	cr->stroke();
		// }

		uint16_t bands=_beatController->getAnalysers().at(idx)->getBands();
		for(uint16_t i=0;i<bands;i++)
		{
			//Draw the function itself
			cr->move_to( ((double)i/(bands-1))*((double)width/2.0) + (((double)width/2.0)*idx), (double)height );
			cr->line_to( ((double)i/(bands-1))*((double)width/2.0) + (((double)width/2.0)*idx), (double)height- (_beatController->getAnalysers().at(idx)->getBand(i)->getNewest()/_beatController->getAnalysers().at(idx)->getMaxBandValue())*(double)height);
			cr->stroke();
		}

		cr->set_source_rgba(1, 0, 0, 0.8);
		cr->arc(0+(((double)width/2.0)*idx), ((double)height-(_beatController->getAnalysers().at(idx)->getBand(0)->getAllTimeMaximumRaw()/_beatController->getAnalysers().at(idx)->getMaxBandValue())*(double)height),
			1.5, 0, 2*M_PI);
		cr->fill();
    	for(uint16_t i=1;i<bands;i++)
    	{
    		cr->arc( ((double)i/(bands-1)*(double)width/2.0) + (((double)width/2.0)*idx), (double)height-(_beatController->getAnalysers().at(idx)->getBand(i)->getAllTimeMaximumRaw()/_beatController->getAnalysers().at(idx)->getMaxBandValue())*(double)height,
			1.5, 0, 2*M_PI);
			cr->fill();
		}
		

		// cr->set_source_rgba(0,1,0,0.8);
		// cr->set_line_width(5);
		// cr->move_to((25.0 + ((double)width/2.0)*idx), (double)height/3.0);
		// cr->line_to((25.0 + ((double)width/2.0)*idx) + _beatController->getAnalysers().at(idx)->getMagSpectrum(50, 2000)*75.0, (double)height/3.0);
		// cr->stroke();

		cr->set_source_rgba(0,1,0,0.8);
		cr->set_line_width(5);
		cr->move_to((160.0 + ((double)width/2.0)*idx), (double)height/3.0);
		cr->line_to((160.0 + ((double)width/2.0)*idx) + ((_beatController->getBuffers().at(idx)->pwr() / _beatController->getBuffers().at(idx)->max_pwr())*75.0) , (double)height/3.0);
		cr->stroke();

		cr->set_source_rgba(1,0,0, (_beatController->getBuffers().at(idx)->pwr() / _beatController->getBuffers().at(idx)->max_pwr()));
		cr->arc((double)width/4.0 + ((double)width/2.0)*idx, (double)height/2.0, 20, 0, 2*M_PI);
		cr->fill();

		m_fps.stop();
		time_elapsed += m_fps.elapsed();
		m_fps.start();
		fps_counter ++;
		if(fps_counter >= 10)
		{
			fps_time = 10.0/time_elapsed;
			time_elapsed = 0;
			fps_counter = 0;
		}
		cr->save();
		cr->set_source_rgba(0.25, 0.25, 0.25, 0.85);
		cr->select_font_face("Cairo", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_NORMAL);
		cr->set_font_size(10);
		cr->move_to(width-66, 20);
		std::stringstream ss;
		ss << std::fixed << std::setprecision(2) << fps_time;
		std::string str("fps: " + ss.str());
		cr->show_text(str.c_str());
		cr->restore();
	}

	return true;
}

void SpektrumViz::soundProcessingDone()
{
		// force our program to redraw the entire clock.
	Glib::RefPtr<Gdk::Window> win = get_window();
	if (win)
	{
		Gdk::Rectangle r(0, 0, get_allocation().get_width(),
		        get_allocation().get_height());
		win->invalidate_rect(r, false);
	}
}