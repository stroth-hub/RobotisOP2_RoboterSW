/*
 * VisionMode.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include "VisionMode.h"
#include "Action.h"
#include "ColorFinder.h"
#include "LinuxActionScript.h"

namespace Robot
{

int VisionMode::Play(int color)
{
    static int old_color = 0, color_count = 0;

    if(old_color != color || color == 0)
    {
        old_color = color;
        color_count = 0;
    }
    else
        color_count++;
    if(color_count < 15) return 1;
	
    switch(color)
    {
    case (RED):
        std::system("espeak -vde \"Das ist die Farbe rot\"");
	sleep(2);
        break;
    case (YELLOW):
        std::system("espeak -vde \"Das ist die Farbe gelb\"");
	sleep(2);
        break;
    case (BLUE):
        std::system("espeak -vde \"Das ist die Farbe blau\"");
	sleep(2);
        break;
    case (RED|YELLOW):
        break;
    case (RED|BLUE):
        break;
    case (BLUE|YELLOW):
        break;
    case (RED|YELLOW|BLUE):
        break;
    default:
	std::system("espeak -vde \"Keine Farbe erkannt\"");
	sleep(2);
	break;
    }

    color_count = 0;
    return 0;
}

}
