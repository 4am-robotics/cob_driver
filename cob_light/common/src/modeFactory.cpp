/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <modeFactory.h>
#include <staticMode.h>
#include <flashMode.h>
#include <breathMode.h>
#include <breathColorMode.h>
#include <fadeColorMode.h>
#include <sequenceMode.h>
#include <circleColorMode.h>
#include <sweepColorMode.h>
#include <distApproxMode.h>
#include <glowColorMode.h>
#include <xmasMode.h>
#include <kitMode.h>
#include <turnIndicatorMode.h>

ModeFactory::ModeFactory()
{
}
ModeFactory::~ModeFactory()
{
}

boost::shared_ptr<Mode> ModeFactory::create(cob_light::LightMode requestMode, IColorO* colorO)
{
    boost::shared_ptr<Mode> mode;
    color::rgba color;
    color.r = requestMode.colors[0].r;
    color.g = requestMode.colors[0].g;
    color.b = requestMode.colors[0].b;
    color.a = requestMode.colors[0].a;

    switch(requestMode.mode)
    {
    case cob_light::LightModes::STATIC:
        mode.reset(new StaticMode(color, requestMode.priority, requestMode.frequency,\
                                  requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::FLASH:
        mode.reset(new FlashMode(color, requestMode.priority, requestMode.frequency,\
                                 requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::BREATH:
        mode.reset(new BreathMode(color, requestMode.priority, requestMode.frequency,\
                                  requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::BREATH_COLOR:
        mode.reset(new BreathColorMode(color, requestMode.priority, requestMode.frequency,\
                                       requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::FADE_COLOR:
        mode.reset(new FadeColorMode(color, requestMode.priority, requestMode.frequency,\
                                     requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::SEQ:
    {
        std::vector<seq_t> seqs;
        for(size_t i = 0; i < requestMode.sequences.size(); i++)
        {
            seq_t seq;
            seq.color.r = requestMode.sequences[i].color.r;
            seq.color.g = requestMode.sequences[i].color.g;
            seq.color.b = requestMode.sequences[i].color.b;
            seq.color.a = requestMode.sequences[i].color.a;
            seq.holdtime = requestMode.sequences[i].hold_time;
            seq.crosstime = requestMode.sequences[i].cross_time;
            seqs.push_back(seq);
        }
        mode.reset(new SequenceMode(seqs, requestMode.priority, requestMode.frequency,\
                                    requestMode.pulses, requestMode.timeout));
    }
        break;

    case cob_light::LightModes::CIRCLE_COLORS:
    {
        std::vector<color::rgba> colors;
        if(requestMode.colors.empty())
        {
            colors.push_back(color);
        }
        else
        {
            for(size_t i = 0; i < requestMode.colors.size(); i++)
            {
                color.r = requestMode.colors[i].r;
                color.g = requestMode.colors[i].g;
                color.b = requestMode.colors[i].b;
                color.a = requestMode.colors[i].a;
                colors.push_back(color);
            }
        }
        mode.reset(new CircleColorMode(colors, colorO->getNumLeds(), requestMode.priority, requestMode.frequency,\
                                       requestMode.pulses, requestMode.timeout));
    }
        break;

    case cob_light::LightModes::SWEEP:
    {
        std::vector<color::rgba> colors;
        if(requestMode.colors.empty())
        {
            colors.push_back(color);
        }
        else
        {
            for(size_t i = 0; i < requestMode.colors.size(); i++)
            {
                color.r = requestMode.colors[i].r;
                color.g = requestMode.colors[i].g;
                color.b = requestMode.colors[i].b;
                color.a = requestMode.colors[i].a;
                colors.push_back(color);
            }
        }
        mode.reset(new SweepColorMode(colors, colorO->getNumLeds(), requestMode.priority, requestMode.frequency,
                                      requestMode.pulses, requestMode.timeout));
    }
        break;

    case cob_light::LightModes::DIST_APPROX:
        mode.reset(new DistApproxMode(colorO->getNumLeds(), requestMode.priority, requestMode.frequency,
                                      requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::GLOW:
        mode.reset(new GlowColorMode(color, requestMode.priority, requestMode.frequency,\
                                     requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::XMAS:
        mode.reset(new XMasMode(colorO->getNumLeds(), requestMode.priority, requestMode.frequency,\
                                     requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::KIT:
        mode.reset(new KitMode(color, colorO->getNumLeds(), requestMode.priority, requestMode.frequency,
                                      requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::TURN_LEFT:
        mode.reset(new TurnIndicatorMode(color, colorO->getNumLeds(), -1, requestMode.priority, requestMode.frequency,
                                      requestMode.pulses, requestMode.timeout));
        break;

    case cob_light::LightModes::TURN_RIGHT:
        mode.reset(new TurnIndicatorMode(color, colorO->getNumLeds(), 1, requestMode.priority, requestMode.frequency,
                                      requestMode.pulses, requestMode.timeout));
        break;

    default:
        mode.reset();
    }

    return mode;
}

boost::shared_ptr<Mode> ModeFactory::create(std::string requestMode, color::rgba color)
{
    boost::shared_ptr<Mode> mode;

    if(requestMode == "Static" || requestMode == "static" || requestMode == "STATIC")
    {
        mode.reset(new StaticMode(color));
    }
    else if(requestMode == "Flash" || requestMode == "flash" || requestMode == "FLASH")
    {
        mode.reset(new FlashMode(color));
    }
    else if(requestMode == "Breath" || requestMode == "breath" || requestMode == "BREATH")
    {
        mode.reset(new BreathMode(color));
    }
    else if(requestMode == "BreathColor" || requestMode == "BreathColor" || requestMode == "BreathColor" ||
            requestMode == "Breath_Color" || requestMode == "breath_color" || requestMode == "BREATH_COLOR")
    {
        mode.reset(new BreathColorMode(color));
    }
    else if(requestMode == "FadeColor" || requestMode == "fadecolor" || requestMode == "FADECOLOR" ||
            requestMode == "Fade_Color" || requestMode == "fade_color" || requestMode == "FADE_COLOR")
    {
        mode.reset(new FadeColorMode(color));
    }
    else
        mode.reset();

    return mode;
}

int ModeFactory::type(Mode *mode)
{
    int ret;
    if (mode == NULL)
        ret = cob_light::LightModes::NONE;
    else if(dynamic_cast<StaticMode*>(mode) != NULL)
        ret = cob_light::LightModes::STATIC;
    else if(dynamic_cast<FlashMode*>(mode) != NULL)
        ret = cob_light::LightModes::FLASH;
    else if(dynamic_cast<BreathMode*>(mode) != NULL)
        ret = cob_light::LightModes::BREATH;
    else if(dynamic_cast<BreathColorMode*>(mode) != NULL)
        ret = cob_light::LightModes::BREATH_COLOR;
    else if(dynamic_cast<FadeColorMode*>(mode) != NULL)
        ret = cob_light::LightModes::FADE_COLOR;
    else if(dynamic_cast<SequenceMode*>(mode) != NULL)
        ret = cob_light::LightModes::SEQ;
    else if(dynamic_cast<CircleColorMode*>(mode) != NULL)
        ret = cob_light::LightModes::CIRCLE_COLORS;
    else if(dynamic_cast<SweepColorMode*>(mode) != NULL)
        ret = cob_light::LightModes::SWEEP;
    else if(dynamic_cast<DistApproxMode*>(mode) != NULL)
        ret = cob_light::LightModes::DIST_APPROX;
    else if(dynamic_cast<GlowColorMode*>(mode) != NULL)
        ret = cob_light::LightModes::GLOW;
    else if(dynamic_cast<XMasMode*>(mode) != NULL)
        ret = cob_light::LightModes::XMAS;
    else if(dynamic_cast<KitMode*>(mode) != NULL)
        ret = cob_light::LightModes::KIT;
    else if(dynamic_cast<TurnIndicatorMode*>(mode) != NULL)
        ret = cob_light::LightModes::TURN_LEFT;
    else if(dynamic_cast<TurnIndicatorMode*>(mode) != NULL)
        ret = cob_light::LightModes::TURN_RIGHT;
    else
        ret = cob_light::LightModes::NONE;

    return ret;
}
