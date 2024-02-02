#include <iostream>
#include "libstruct.h"
#include <ncurses.h>

using namespace std;
class CursePrinter
{
    public:
        CursePrinter(WINDOW * win);
        void Print1(Glider g);
        void Print2(Glider g, PID h, PID p, PID b);
    private:
        WINDOW * curwin;
        Glider gld;
};

CursePrinter::CursePrinter(WINDOW * win)
{
    curs_set(0);
    curwin = win;
}

void CursePrinter::Print1(Glider g)
{
    mvwprintw(curwin,1,1,"depth  : %0.4f  ",g.currentDepth);
    wrefresh(curwin); 
}

void CursePrinter::Print2(Glider g, PID h, PID p, PID b)
{
    mvwprintw(curwin,1,1,"depth  : %0.4f  ",g.currentDepth);    
    mvwprintw(curwin,2,1,"pitch  : %0.4f  ",g.currentPitch);
    mvwprintw(curwin,3,1,"heading: %0.4f  ",g.teta_terukur);
    if (g.pose == 1)
        mvwprintw(curwin,4,1,"POSE : ASCENDING ");
    else 
    if (g.pose == 2)
        mvwprintw(curwin,4,1,"POSE : STAYSTILL");
    else
        mvwprintw(curwin,4,1,"POSE : DESCENDING");

    mvwprintw(curwin,6,1,"PID1 : %0.4f  ",h.output);
    mvwprintw(curwin,7,1,"PID2 : %0.4f  ",p.output);
    mvwprintw(curwin,8,1,"PID3 : %0.4f  ",b.output);
    
    wrefresh(curwin);
}