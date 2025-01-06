#ifndef MENU_H
#define MENU_H

#include <stdint.h>

typedef void (*MenuAction)(void);

typedef struct MenuItem
{
    char name[16];
    struct MenuItem *parent;
    struct MenuItem **children;
    uint8_t numChildren;
    char action[6];
    uint16_t value;
} MenuItem;

void initMenu(void);
void moveSibling(int direction);
void moveParentChild(int direction);
MenuItem *getCurrentMenuItem(void);

#endif // MENU_H
