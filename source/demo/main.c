
#include "demo\spDemo.h"

typedef void (*demoFunc)();

extern demoFunc Joints;
extern demoFunc Convex;
extern demoFunc Chains;
extern demoFunc Cloth;

static void
PrintMenu()
{
    fprintf(stdout, "********************************************************************************\n");
    fprintf(stdout, "Please select a demo:\n");
    fprintf(stdout, "1: joints - Demonstrates each of the 8 different constraint types.\n");
    fprintf(stdout, "2: concave - Concave shapes using multiple shapes with a single rigid body.\n");
    fprintf(stdout, "3: chains - A chain built using distance constraints and boxes.\n");
    fprintf(stdout, "4: cloth  - Simple 2D cloth built using spring constraints.\n");
    fprintf(stdout, "0: quit\n");
    fprintf(stdout, "\n");
}

static int 
SelectDemo()
{
    PrintMenu();

    int demo;
#if (_MSC_VER)
    scanf_s(" %d", &demo);
#else
    scanf(" %d", &demo);
#endif


    switch(demo)
    {
    case 1:
        fprintf(stdout, "\n");
        fprintf(stdout, "Controls: R - Reset the demo\n");
        fprintf(stdout, "          G - Turn gravity on/off\n");
        fprintf(stdout, "\n");
        Joints();
        break;
    case 2:
        fprintf(stdout, "\n");
        fprintf(stdout, "Controls: R - Reset the demo\n");
        fprintf(stdout, "          G - Turn gravity on/off\n");
        fprintf(stdout, "\n");
        Convex();
        break;
    case 3:
        fprintf(stdout, "\n");
        fprintf(stdout, "Controls: R - Reset the demo\n");
        fprintf(stdout, "          G - Turn gravity on/off\n");
        fprintf(stdout, "\n");
        Chains();
        break;
    case 4:
        fprintf(stdout, "\n");
        fprintf(stdout, "Controls: R - Reset the demo\n");
        fprintf(stdout, "          G - Turn gravity on/off\n");
        fprintf(stdout, "\n");
        Cloth();
        break;
    default:
        return 0;
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "********************************************************************************\n");
    fprintf(stdout, "\n");

    return 1;
}

int main()
{
    int run = 1;
    while (run = SelectDemo()) {}

    return 0;
}