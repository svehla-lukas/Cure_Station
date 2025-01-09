#include <menu.h>
#include <stdio.h>
#include <string.h>

// Global variables
MenuItem *currentMenuItem;

uint16_t timeValue = 60;
uint16_t tempValue = 25;

// Initialize the menu structure
void initMenu(void)
{
    const uint16_t defaultTempValue = 20;
    const uint16_t defaultTimeValue = 60;
    // Virtual root node for Level 0
    static MenuItem root = {"Root", NULL, NULL, 0};

    // Level 0
    static MenuItem wash = {"Wash", &root, NULL, 0};
    static MenuItem dry = {"Dry", &root, NULL, 0};
    static MenuItem cure = {"Cure", &root, NULL, 0};

    static MenuItem *mainMenu[3]; // array of pointers
    mainMenu[0] = &dry;
    mainMenu[1] = &cure;
    mainMenu[2] = &wash;

    root.children = mainMenu;
    root.numChildren = 3;

    // Level 1 - Dry
    static MenuItem dryStart = {"Start", &dry, NULL, 0};
    static MenuItem dryTemp = {"Temperature", &dry, NULL, 0};
    static MenuItem dryTime = {"Time", &dry, NULL, 0};

    static MenuItem *dryChildren[3];
    dryChildren[0] = &dryStart;
    dryChildren[1] = &dryTemp;
    dryChildren[2] = &dryTime;

    // Assign dry's children and set their parent pointers
    dry.children = dryChildren;
    dry.numChildren = 3;
    dryChildren[0]->parent = &dry;
    dryChildren[1]->parent = &dry;
    dryChildren[2]->parent = &dry;

    // Level 1 - Cure
    static MenuItem cureStart = {"Start", &cure, NULL, 0};
    static MenuItem cureTemp = {"Temperature", &cure, NULL, 0};
    static MenuItem cureTime = {"Time", &cure, NULL, 0};

    static MenuItem *cureChildren[3];
    cureChildren[0] = &cureStart;
    cureChildren[1] = &cureTemp;
    cureChildren[2] = &cureTime;

    // Assign cure's children and set their parent pointers
    cure.children = cureChildren;
    cure.numChildren = 3;
    cureChildren[0]->parent = &cure;
    cureChildren[1]->parent = &cure;
    cureChildren[2]->parent = &cure;

    // Level 1 - Wash
    static MenuItem washStart = {"Start", &wash, NULL, 0};
    static MenuItem washTime = {"Time", &wash, NULL, 0};

    static MenuItem *washChildren[2];
    washChildren[0] = &washStart;
    washChildren[1] = &washTime;

    // Assign wash's children and set their parent pointers
    wash.children = washChildren;
    wash.numChildren = 2;
    washChildren[0]->parent = &wash;
    washChildren[1]->parent = &wash;

    // Level 2 - Dry Start
    static MenuItem dryStartValue =
        {"Start: ", &dryStart, NULL, 0, "start", 30};
    static MenuItem *dryStartValueChildren[1];
    dryStartValueChildren[0] = &dryStartValue;

    // Assign dryStart's children and set their parent pointers
    dryStart.children = dryStartValueChildren;
    dryStart.numChildren = 1;
    dryStartValue.parent = &dryStart;

    // Level 2 - Dry Temp
    static MenuItem dryTempValue = {"Temp: ", &dryTemp, NULL, 0, "temp",
                                    defaultTempValue};
    static MenuItem *dryTempValueChildren[1];
    dryTempValueChildren[0] = &dryTempValue;

    // Assign dryTemp's children and set their parent pointers
    dryTemp.children = dryTempValueChildren;
    dryTemp.numChildren = 1;
    dryTempValue.parent = &dryTemp;

    // Level 2 - Dry Time
    static MenuItem dryTimeValue = {"Time: ", &dryTime, NULL, 0, "time",
                                    defaultTimeValue};
    static MenuItem *dryTimeValueChildren[1];
    dryTimeValueChildren[0] = &dryTimeValue;

    // Assign dryTime's children and set their parent pointers
    dryTime.children = dryTimeValueChildren;
    dryTime.numChildren = 1;
    dryTimeValue.parent = &dryTime;

    // Level 2 - Cure Start
    static MenuItem cureStartValue = {"Start: ", &cureStart, NULL, 0, "start",
                                      30};
    static MenuItem *cureStartValueChildren[1];
    cureStartValueChildren[0] = &cureStartValue;

    // Assign cureStart's children and set their parent pointers
    cureStart.children = cureStartValueChildren;
    cureStart.numChildren = 1;
    cureStartValue.parent = &cureStart;

    // Level 2 - Cure Temp
    static MenuItem cureTempValue = {"Temp: ", &cureTemp, NULL, 0, "temp",
                                     defaultTempValue};
    static MenuItem *cureTempValueChildren[1];
    cureTempValueChildren[0] = &cureTempValue;

    // Assign cureTemp's children and set their parent pointers
    cureTemp.children = cureTempValueChildren;
    cureTemp.numChildren = 1;
    cureTempValue.parent = &cureTemp;

    // Level 2 - Cure Time
    static MenuItem cureTimeValue = {"Time: ", &cureTime, NULL, 0, "time",
                                     defaultTimeValue};
    static MenuItem *cureTimeValueChildren[1];
    cureTimeValueChildren[0] = &cureTimeValue;

    // Assign cureTime's children and set their parent pointers
    cureTime.children = cureTimeValueChildren;
    cureTime.numChildren = 1;
    cureTimeValue.parent = &cureTime;

    // Set initial menu item to root
    currentMenuItem = &root;

    // Level 2 - WashStart
    static MenuItem washStartValue = {"Start: ", &washStart, NULL, 0, "start",
                                      30};
    static MenuItem *washStartValueChildren[1];
    washStartValueChildren[0] = &washStartValue;

    // Assign washStart's children and set their parent pointers
    washStart.children = washStartValueChildren;
    washStart.numChildren = 1;
    washStartValue.parent = &washStart;

    // Level 2 - Wash Time
    static MenuItem washTimeValue = {"Time: ", &washTime, NULL, 0, "time",
                                     defaultTimeValue};
    static MenuItem *washTimeValueChildren[1];
    washTimeValueChildren[0] = &washTimeValue;

    // Assign washTime's children and set their parent pointers
    washTime.children = washTimeValueChildren;
    washTime.numChildren = 1;
    washTimeValue.parent = &washTime;
}

// Move through the menu
void moveSibling(int direction)
{
    // Definice konstant a kroků
    const uint16_t stepTemp = 5;
    const uint16_t maxTemp = 80;
    const uint16_t minTemp = 15;

    const uint16_t stepTime = 15;
    const uint16_t maxTime = 600;
    const uint16_t minTime = 15;

    if (currentMenuItem->parent)
    {
        MenuItem *parent = currentMenuItem->parent;

        for (uint8_t i = 0; i < parent->numChildren; i++)
        {
            if (parent->children[i] == currentMenuItem)
            {
                // move between children
                if (direction > 0 && i + 1 < parent->numChildren)
                {
                    currentMenuItem = parent->children[i + 1];
                }
                else if (direction < 0 && i > 0)
                {
                    currentMenuItem = parent->children[i - 1];
                }

                // take action if exist
                if (currentMenuItem->action[0] != '\0')
                {
                    if (strcmp(currentMenuItem->action, "time") == 0)
                    {
                        if (direction > 0)
                        {
                            currentMenuItem->value = (currentMenuItem->value + stepTime > maxTime) ? maxTime : currentMenuItem->value + stepTime;
                        }
                        else
                        {
                            currentMenuItem->value = (currentMenuItem->value - stepTime < minTime) ? minTime : currentMenuItem->value - stepTime;
                        }
                    }
                    else if (strcmp(currentMenuItem->action, "temp") == 0)
                    {
                        if (direction > 0)
                        {
                            currentMenuItem->value = (currentMenuItem->value + stepTemp > maxTemp) ? maxTemp : currentMenuItem->value + stepTemp;
                        }
                        else
                        {
                            currentMenuItem->value = (currentMenuItem->value - stepTemp < minTemp) ? minTemp : currentMenuItem->value - stepTemp;
                        }
                    }
                }
                break;
            }
        }
    }
}

void moveParentChild(int direction)
{
    if (direction > 0)
    {
        if (currentMenuItem->children && currentMenuItem->numChildren > 0)
        {
            currentMenuItem = currentMenuItem->children[0];
        }
        else if (currentMenuItem->parent)
        {
            currentMenuItem = currentMenuItem->parent;
        }
    }

    else if (direction < 0)
    {
        if (currentMenuItem->parent)
        {
            currentMenuItem = currentMenuItem->parent;
        }
    }
}

// Get the current menu item
MenuItem *getCurrentMenuItem(void)
{
    return currentMenuItem;
}
