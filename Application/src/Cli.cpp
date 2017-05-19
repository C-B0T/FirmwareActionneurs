/**
 * @file    CLI.cpp
 * @author  Jeremy ROULLAND
 * @date    23 mar. 2017
 * @brief   Command Line Interface class
 */

#include "Cli.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define CLI_BUF_MAX        40

#define CLI_TASK_STACK_SIZE          (256u)
#define CLI_TASK_PRIORITY            (1u)

#define CLI_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static CLI* _cli = NULL;

/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/

CLI* CLI::GetInstance()
{
    // If ???? instance already exists
    if(_cli != NULL)
    {
        return _cli;
    }
    else
    {
        _cli = new CLI();
        return _cli;
    }
}

CLI::CLI()
{
    this->name = "Cli";
    this->taskHandle = NULL;

    // Create task
    xTaskCreate((TaskFunction_t)(&CLI::taskHandler),
                this->name.c_str(),
                CLI_TASK_STACK_SIZE,
                NULL,
                CLI_TASK_PRIORITY,
                NULL);

    this->odometry = Odometry::GetInstance(false);
    this->pc = PositionControl::GetInstance(false);
    this->pg = ProfileGenerator::GetInstance(false);
    this->tp = TrajectoryPlanning::GetInstance(false);
    this->mc = FBMotionControl::GetInstance();

    this->diag = Diag::GetInstance();
}


void CLI::Start()
{
	putchar('>');
}

void CLI::Compute(float32_t period)
{
    static char buf[CLI_BUF_MAX] = {0};
    static uint16_t i = 0;

    char c;
    char *pch;

    c = getchar();

    if(c == '&')
    {
    	printf("!!EMERGENCY STOP!! (NOT IMPLEMENTED!!!!!!!!)");

    }
    else if(c == '(')
    {
        this->diag->Toggle(0);
    }
    else if(c == '_')
    {
        this->diag->Toggle(1);
    	putchar(c);
    }
    else if(c == ')')
    {
    	putchar(c);
    }
    else if(c == '=')
    {
    	putchar(c);
    }
    else if(c == ',')
    {
    	putchar(c);
    }
    else if(c == ';')
    {
    	putchar(c);
    }
    else if(c == ':')
    {
        putchar(c);
        pg->SetAngularVelMax(3.14);
        pg->SetAngularAccMax(3.14);
        pg->SetLinearVelMax(0.4);
        pg->SetLinearAccMax(1.0);
        printf("\r\nAngVel=3.14 AngAcc=3.14 LinVel=0.4 LinAcc=1.0\r\n");
    }
    else if(c == '!')
    {
        putchar(c);
        pg->SetAngularVelMax(12.0);
        pg->SetAngularAccMax(18.0);
        pg->SetLinearVelMax(1.0);
        pg->SetLinearAccMax(2.0);
        printf("\r\nAngVel=12.0 AngAcc=18.0 LinVel=1.0 LinAcc=2.0\r\n");
    }
    else if( ((c >= '0') && (c <= '9')) ||
        ((c >= 'a') && (c <= 'z')) ||
        ((c >= 'A') && (c <= 'Z')) ||
         (c == '.') || (c == ' ')  ||
         (c == '-')  )
    {
        if(i < (CLI_BUF_MAX-2))
        {
            putchar(c);
            buf[i] = c;
            i++;
            buf[i] = '\0';
        }

    }
    else if(c == '\b')
    {
        putchar('\b');
        putchar(' ');
        putchar('\b');
        if(i>0)
            i--;
        buf[i] = '\0';
    }
    else if( (c == '\r') || (c == '\n') )
    {
        pch = strtok (buf," ");

        if(strcmp(pch,"help") == 0)
        {
            printf("\r\n## help (v0.1):\r\n");
            printf(" Shortcut:\r\n");
            printf(" - &            \tEmergency stop\r\n");
            printf(" - (            \tToggle traces\r\n");
            printf(" Command:\r\n");
            printf(" - status             \tGet modules status\r\n");
            printf(" - enable             \tEnable motion control\r\n");
            printf(" - disable            \tDisable motion control\r\n");
            printf(" - golin <l>          \tGo Linear\r\n");
            printf(" - goang <a>          \tGo Angular\r\n");
            printf(" - goto <x> <y>       \tGo to X,Y\r\n");
            printf(" - getodo             \tGet odometry X,Y,O\r\n");
            printf(" - setodo <x> <y> <o> \tSet odometry X,Y,O\r\n");
            printf(" - setvellin <v>      \tSet velocity linear\r\n");
            printf(" - setvelang <v>      \tSet velocity angular\r\n");
            printf(" - setacclin <a>      \tSet acceleration linear\r\n");
            printf(" - setaccang <a>      \tSet acceleration angular\r\n");
            printf(" - free               \tFreewheel\r\n");
            printf(" - stop <%%>          \tStop %% Brake\r\n");
            printf(" = \r\n");
            printf(" - GoLin <l>          \tGo Linear (mm)\r\n");
            printf(" - GoAng <a>          \tGo Angular (1/10deg)\r\n");
            printf(" - Goto <x> <y>       \tGo to X,Y (mm)\r\n");
            printf(" - Stop               \tStop motion\r\n");
            printf(" - Test               \tGoLin(500), GoAng(1800), GoLin(500), GoAng(0)\r\n");
        }
        else if(strcmp(pch,"enable") == 0)
        {
            mc->Enable();
            printf("\r\nenable");
        }
        else if(strcmp(pch,"disable") == 0)
        {
            mc->Disable();
            printf("\r\ndisable");
        }
        else if(strcmp(pch,"golin") == 0)
        {
            float l;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                l = strtof(pch, NULL);
            else
                l = 0.0;

            printf("\r\ngolin %.3f", l);
            tp->goLinear(l);
            //pc->SetLinearPosition(l); // ATTENTION : Absolute value
            //pc->Test();
            //pg->StartLinearVelocity(l);
        }
        else if(strcmp(pch,"GoLin") == 0)
        {
            int16_t d;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                d = strtol(pch, NULL, 10);
            else
                d = 0;

            printf("\r\nGoLin %d", d);
            mc->GoLin(d);
        }
        else if(strcmp(pch,"goang") == 0)
        {
            float a;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                a = strtof(pch, NULL);
            else
                a = 0.0;

            printf("\r\ngoang %.3f", a);
            tp->goAngular(a);
            //pc->SetAngularPosition(a);
            //pc->Test();
            //pg->StartAngularVelocity(a);
        }
        else if(strcmp(pch,"GoAng") == 0)
        {
            int16_t a;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                a = strtol(pch, NULL, 10);
            else
                a = 0;

            printf("\r\nGoAng %d", a);
            mc->GoAng(a);
        }
        else if(strcmp(pch,"goto") == 0)
        {
            float x, y;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                x = strtof(pch, NULL);
            else
                x = 0.0;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                y = strtof(pch, NULL);
            else
                y = 0.0;

            printf("\r\ngoto %.3f %.3f", x, y);
            tp->gotoXY(x,y);
        }
        else if(strcmp(pch,"Goto") == 0)
        {
            int16_t x, y;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                x = strtol(pch, NULL, 10);
            else
                x = 0;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                y = strtol(pch, NULL, 10);
            else
                y = 0;

            printf("\r\nGoto %d %d", x, y);
            mc->Goto(x,y);
        }
        else if(strcmp(pch,"getodo") == 0)
        {
            robot_t r;
            odometry->GetRobot(&r);
            printf("\r\ngetodo: %ld\t%ld\t%ld", r.Xmm, r.Ymm, (int32_t)(r.Odeg*10.0));
        }
        else if(strcmp(pch,"setodo") == 0)
        {
            float x, y, o;

            pch = strtok (NULL, " ");
            if(pch != NULL)
                x = strtof(pch, NULL);
            else
                x = 0.0;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                y = strtof(pch, NULL);
            else
                y = 0.0;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                o = strtof(pch, NULL);
            else
                o = 0.0;

            printf("\r\nsetodo %.3f %.3f %.3f", x, y, o);
            odometry->SetXYO(x,y,o);
        }
        else if(strcmp(pch,"free") == 0)
        {
        	tp->freewheel();
        }
        else if(strcmp(pch,"stop") == 0)
        {
            float bk = 1.0;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                bk = strtof(pch, NULL);
            printf("\r\nstop (%.1f Brake)", bk);
            tp->stop();
        }
        else if(strcmp(pch,"Stop") == 0)
        {
            printf("\r\nStop");
            mc->Stop();
        }
        else if(strcmp(pch,"checkup") == 0)
        {
            // TODO: Checkup
            printf("\r\ncheckup");
        }
        else if(strcmp(pch,"safeguard") == 0)
        {
        	mc->ToggleSafeguard();
            printf("\r\nsafeguard=%d", mc->GetSafeguard());
        }
        else if(strcmp(pch,"status") == 0)
        {
            printf("\r\nStatus:\r\n");
            printf(" safeguard:%d\r\n", mc->GetSafeguard());
            printf(" mc:0x%04x\r\n", mc->GetStatus());
            printf(" tp:0x%04x\r\n", tp->GetStatus());
            printf(" pg:0x%04x\r\n", pg->GetStatus());
            printf(" pc:0x%04x\r\n", pc->GetStatus());
            printf(" od:0x%04x\r\n", odometry->GetStatus());
        }
        else if(strcmp(pch,"mc") == 0)
        {
            pch = strtok (NULL, " ");
            if(pch != NULL)
            {
            	if(strcmp(pch,"dis") == 0)
            	{
                    printf("\r\nmc disable");
                    mc->Disable();
            	}
            	else
            	{
                    printf("\r\nmc enable");
                    mc->Enable();
            	}
            }
            else
            {
                printf("\r\nmc enable");
                mc->Enable();
            }
        }
        else if(strcmp(pch,"setvellin") == 0)
        {
            float v;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                v = strtof(pch, NULL);
            else
                v = 0.0;

            printf("\r\nsetvellin %.3f", v);
            pg->SetLinearVelMax(v);
        }
        else if(strcmp(pch,"setvelang") == 0)
        {
            float v;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                v = strtof(pch, NULL);
            else
                v = 0.0;

            printf("\r\nsetvelang %.3f", v);
            pg->SetAngularVelMax(v);
        }
        else if(strcmp(pch,"setacclin") == 0)
        {
            float a;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                a = strtof(pch, NULL);
            else
                a = 0.0;

            printf("\r\nsetacclin %.3f", a);
            pg->SetLinearAccMax(a);
        }
        else if(strcmp(pch,"setaccang") == 0)
        {
            float a;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                a = strtof(pch, NULL);
            else
                a = 0.0;

            printf("\r\nsetaccang %.3f", a);
            pg->SetAngularAccMax(a);
        }
        else if(strcmp(pch,"Test") == 0)
        {
            printf("\r\ntest");
            mc->GoLin(500);
            mc->GoAng(1800);
            mc->GoLin(500);
            mc->GoAng(0);
        }
        else if(strcmp(pch,"ki") == 0)
        {
            float kp;
            pch = strtok (NULL, " ");
            if(pch != NULL)
                kp = strtof(pch, NULL);
            else
                kp = 0.0;

            printf("\r\nkp %.3f", kp);
            //pc->SetAngularKp(kp);
        }
        else
        {
        	if(i > 0)
        		printf("\r\nBad cmd!!");
        }

        i=0;
        buf[i] = '\0';
        putchar('\r');
        putchar('\n');
        putchar('>');
    } /*else if( (c == '\r') || (c == '\n') )*/

} /* void CLI::Compute() */

void CLI::taskHandler (void* obj)
{
    TickType_t xLastWakeTime;
    TickType_t xFrequency = pdMS_TO_TICKS(CLI_TASK_PERIOD_MS);

    CLI* instance = _cli;
    TickType_t prevTick = 0u,  tick = 0u;

    float32_t period = 0.0f;

    // 0. Delay
    vTaskDelay(pdMS_TO_TICKS(1000u));

    instance->Start();

    // 1. Initialise periodical task
    xLastWakeTime = xTaskGetTickCount();

    // 2. Get tick count
    prevTick = xTaskGetTickCount();

    while(1)
    {
        // 2. Wait until period elapse
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 3. Get tick
        tick = xTaskGetTickCount();

        period = static_cast<float32_t>(tick) -
                 static_cast<float32_t>(prevTick);

        //4. Compute Diag informations
        instance->Compute(period);

        // 5. Set previous tick
        prevTick = tick;
    }
}
