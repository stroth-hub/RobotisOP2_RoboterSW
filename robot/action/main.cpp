/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <cstdlib>

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "ColorFinder.h"
#include "StatusCheck.h"
#include "VisionMode.h"
#include "cmd_process.h"
#include "Action.h"
#include "Head.h"
#include "Walking.h"
#include "MX28.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "LinuxCM730.h"
#include "LinuxActionScript.h"
#include "LinuxDARwIn.h"
#include "soccer.h"

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <limits.h>

#define PORT 5005 
#define MAXLINE 1024

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

int gID = CM730::ID_CM;

int cleanup = 0;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}
void sighandler(int sig)
{
    cleanup = 1;
    //exit(0);
}
void vision(ColorFinder* ball_finder, ColorFinder* red_finder, ColorFinder* blue_finder, ColorFinder* yellow_finder, Image* rgb_output, int& detected_color)
{
	Point2D ball_pos, red_pos, yellow_pos, blue_pos;
	LinuxCamera::GetInstance()->CaptureFrame();
	usleep(10000);
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
	usleep(10000);
	if(StatusCheck::m_cur_mode == VISION)
        {
        	ball_pos = ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            	red_pos = red_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
           	yellow_pos = yellow_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
            	blue_pos = blue_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
          	unsigned char r, g, b;
            	for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            	{	
                	r = 0; g = 0; b = 0;
                	if(ball_finder->m_result->m_ImageData[i] == 1)
                	{
                  		r = 255;
                    		g = 128;
                    		b = 0;
                	}
                	if(red_finder->m_result->m_ImageData[i] == 1)
                	{
                    		if(ball_finder->m_result->m_ImageData[i] == 1)
                    		{
                        		r = 0;
                        		g = 255;
                        		b = 0;
                    		}
                    		else
                    		{
                        		r = 255;
                        		g = 0;
                        		b = 0;
                    		}
                	}
                	if(yellow_finder->m_result->m_ImageData[i] == 1)
                	{
                  		if(ball_finder->m_result->m_ImageData[i] == 1)
                    		{
                     			r = 0;
                        		g = 255;
                        		b = 0;
                    		}
                    		else
                    		{
                        		r = 255;
                        		g = 255;
                        		b = 0;
                    		}
                	}
                	if(blue_finder->m_result->m_ImageData[i] == 1)
                	{
                  		if(ball_finder->m_result->m_ImageData[i] == 1)
                    		{
                        		r = 0;
                        		g = 255;
                        		b = 0;
                    		}
                    		else
                    		{
                        		r = 0;
                        		g = 0;
                        		b = 255;
                    		}
                	}

                	if(r > 0 || g > 0 || b > 0)
                	{
                  		rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = r;
                    		rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = g;
                    		rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = b;
                	}
            	}
        	//streamer->send_image(rgb_output);
        	detected_color = 0;
            	detected_color |= (red_pos.X == -1)? 0 : VisionMode::RED;
            	detected_color |= (yellow_pos.X == -1)? 0 : VisionMode::YELLOW;
            	detected_color |= (blue_pos.X == -1)? 0 : VisionMode::BLUE;
		printf("%d\n",detected_color);
            				
	}
}
void soccer(ColorFinder* ball_finder, BallTracker& tracker, mjpg_streamer* streamer, BallFollower& follower, Image* rgb_output)
{
	int _ball_found = 0;
        LinuxCamera::GetInstance()->CaptureFrame();
      	memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        if(StatusCheck::m_cur_mode == SOCCER)
        {
          	//tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
           	_ball_found = tracker.SearchAndTracking(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
            	for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            	{
                	if(ball_finder->m_result->m_ImageData[i] == 1)
                	{
                    		rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                    		rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
                 		rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                	}
            	}
        	streamer->send_image(rgb_output);
        	//if(StatusCheck::m_is_started == 0)continue;
       		if(Action::GetInstance()->IsRunning() == 0 && StatusCheck::m_soccer_sub_mode == SOCCER_PLAY)
            	{
                	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                        if(Walking::GetInstance()->IsRunning() == false && _ball_found != 1)
			{
            			Walking::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
            			Walking::GetInstance()->Start();
            		}
                	if(_ball_found == 1)
                	{
                    		follower.Process(tracker.ball_position);
                    		if(follower.KickBall != 0)
                    		{
                        		Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                        		Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                        		if(follower.KickBall == -1)
                        		{
                            			Action::GetInstance()->Start(12);   // RIGHT KICK
						while(Action::GetInstance()->IsRunning() == true) usleep(8000);
                            			fprintf(stderr, "RightKick! \n");
			                }
                        		else if(follower.KickBall == 1)
                        		{
                            			Action::GetInstance()->Start(13);   // LEFT KICK
						while(Action::GetInstance()->IsRunning() == true) usleep(8000);
                           			fprintf(stderr, "LeftKick! \n");
                        		}
                		}
                	}
                	else if(_ball_found == -1)
                	{
            			Walking::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            			Walking::GetInstance()->A_MOVE_AMPLITUDE = 10.0;
                	}	
                	else
                	{
            			Walking::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            			Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
                	}
         	}
	}
}

int main()
{
	printf("Init Main\n");
	signal(SIGABRT, &sighandler);
    	signal(SIGTERM, &sighandler);
    	signal(SIGQUIT, &sighandler);
    	signal(SIGINT, &sighandler);

	printf("TCP Init\n");
   	int sockfd; 
    	char buffer[MAXLINE]; 
	char buffer_tmp[MAXLINE];
    	struct sockaddr_in servaddr, cliaddr; 
    	cliaddr.sin_addr.s_addr = inet_addr("192.168.137.85");
       
    	// Creating socket file descriptor 
    	if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) 
	{ 
        	perror("socket creation failed\n"); 
        	exit(EXIT_FAILURE); 
	}
       
    	memset(&servaddr, 0, sizeof(servaddr)); 
    	memset(&cliaddr, 0, sizeof(cliaddr)); 
       
    	// Filling server information 
    	servaddr.sin_family    = AF_INET; // IPv4 
    	servaddr.sin_addr.s_addr = inet_addr("192.168.137.1"); 
    	servaddr.sin_port = htons(PORT); 
       
    	//Bind the socket with the server address 
    	//if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
        //    sizeof(servaddr)) < 0 ) 
    	//{ 
        //	perror("bind failed\n"); 
        //	exit(EXIT_FAILURE); 
    	//}       
    	socklen_t len;
    	int n;   
    	len = sizeof(cliaddr);  //len is value/result 
	
	if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		printf("Connection Failed\n");
	}
	printf("Connection Success\n");


			
	
	printf("Framework Init\n");
    	minIni* ini = new minIni(INI_FILE_PATH);
    	change_current_dir();
	Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	int init_attempt = 0;
	int const init_attempt_max = 10;
	while(cm730.Connect() == false)
	{
		init_attempt++;
		if(init_attempt > init_attempt_max)
		{
			printf("Fail to connect CM-730!\n");
			return 0;
		}
	}
	printf("Success to connect CM-730!\n");
	cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(255,125,0), 0);

	printf("Motion Init\n");
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		init_attempt = 0;
		while(MotionManager::GetInstance()->Reinitialize() == false)
		{
			init_attempt++;
			if(init_attempt > init_attempt_max)
			{
				printf("Fail to reinitialize Motion Manager!\n");
				return 0;
			}		
		}
	}
	printf("Initialize Motion Manager!\n");
	Walking::GetInstance()->LoadINISettings(ini);
	MotionManager::GetInstance()->LoadINISettings(ini);
	MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
        MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	Point2D ball_pos, red_pos, yellow_pos, blue_pos;
	LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
	motion_timer->Start();
	MotionManager::GetInstance()->SetEnable(true);
	printf("SetEnable\n");
	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
        bool action_b = false;
	action_b = Action::GetInstance()->Start(1);    /* Init(stand up) pose */
	while(Action::GetInstance()->IsRunning() == true) usleep(8000);
	if(action_b == false)
	{
		printf("Init Pose konnte nicht ausgeführt werden! (cpp)\n");
	}
	else
	{
		printf("Init Pose konnte ausgeführt werden! (cpp)\n");
	}


        Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    	LinuxCamera::GetInstance()->Initialize(0);
   	LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    	LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini
        mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

        ColorFinder* ball_finder = new ColorFinder();
        ball_finder->LoadINISettings(ini);
        httpd::ball_finder = ball_finder;

        BallTracker tracker = BallTracker();
        BallFollower follower = BallFollower();

    	ColorFinder* red_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0);
    	red_finder->LoadINISettings(ini, "RED");
    	httpd::red_finder = red_finder;

    	ColorFinder* yellow_finder = new ColorFinder(60, 15, 45, 0, 0.3, 50.0);
    	yellow_finder->LoadINISettings(ini, "YELLOW");
    	httpd::yellow_finder = yellow_finder;

    	ColorFinder* blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
    	blue_finder->LoadINISettings(ini, "BLUE");
    	httpd::blue_finder = blue_finder;

	httpd::ini = ini;

	int* int_err;
	while(Scan(&cm730)!=0)
	{
		if(Scan(&cm730)==20)
		{
			init_attempt = 0;
			while(cm730.Connect() == false)
			{
				init_attempt++;
				if(init_attempt > init_attempt_max)
				{
					printf("Fail to connect CM-730!\n");
					return 0;
				}
			}
		}
	}    	
	cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(0,255,0), 0);
	while(1)
	{
		send(sockfd, "req_new", strlen("cmd_req"),0);
		n = read(sockfd, (char *)buffer, MAXLINE);    		
		//n = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &cliaddr,&len); 
		buffer[n] = '\0';
		//printf("Client : %s\n", buffer);
		if(cleanup != 0)
		{
			printf("Cleanup\n");
			send(sockfd, "clean_up", strlen("clean_up"),0);
			close(sockfd);
			exit(0);
		}
		else if(strcmp(buffer,"close") == 0)
		{
			printf("Script beendet\n");
			return 0;
		}
		else if(strcmp(buffer,"hallo") == 0)
		{
			std::string line = buffer;
			line = "\""+line+"\"";
			std::system(("espeak -vde "+line).c_str());
		}
		else if(strcmp(buffer,"vision") == 0)
		{	
			cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(255,125,0), 0);
			n = read(sockfd, (char *)buffer, MAXLINE); 
			buffer[n] = '\0';
			strcpy(buffer_tmp, buffer);
			printf("Client : %s\n", buffer_tmp);
			std::string line = buffer_tmp;
			line = "\""+line+"\"";
			std::system(("espeak -vde "+line).c_str());
			StatusCheck::m_cur_mode = VISION;
			StatusCheck::Check(cm730);
			int detected_color = 0;
			while(VisionMode::Play(detected_color))
			{
				vision(ball_finder,red_finder,blue_finder,yellow_finder,rgb_output, detected_color);
			}
			if(Scan(&cm730)==20)cm730.Connect();
			cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(0,255,0), 0);
		}
		else if(strcmp(buffer,"soccer") == 0)
		{
			cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(255,125,0), 0);
			n = read(sockfd, (char *)buffer, MAXLINE); 
			buffer[n] = '\0';
			strcpy(buffer_tmp, buffer);
			printf("Client : %s\n", buffer_tmp);
			std::string line = buffer_tmp;
			line = "\""+line+"\"";
			std::system(("espeak -vde "+line).c_str());
			StatusCheck::m_cur_mode = START;
			while(1)
    			{
        			StatusCheck::Check(cm730);
				soccer(ball_finder, tracker, streamer, follower, rgb_output);
				if(StatusCheck::m_soccer_sub_mode == SOCCER_END)
				{
					//printf("Soccer Ende \n");
					//if(Scan(&cm730)==20)cm730.Connect();
					cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(0,255,0), 0);
					break;
				}
					
			}
		}
		else if(strcmp(buffer,"no_cmd") == 0)
		{
			if(Scan(&cm730)==20)
			{
				printf("cm730 Disconnected \n");
				cm730.Connect();
				Scan(&cm730);
			}
			cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(0,255,0), 0);
		}
		else
		{	
			cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(255,125,0), 0);
			strcpy(buffer_tmp, buffer);
			n = read(sockfd, (char *)buffer, MAXLINE); 
			buffer[n] = '\0';
			printf("Client : %s\n", buffer);
			std::string line = buffer;
			line = "\""+line+"\"";
			LinuxActionScript::ScriptStart(buffer_tmp);
			usleep(1000);
			std::system(("espeak -vde "+line).c_str());
			while(Action::GetInstance()->IsRunning()) usleep(8*1000);
			cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(0,255,0), 0);
		}	    		  
	}
}

