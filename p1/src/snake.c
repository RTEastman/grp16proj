#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h> // for random()
#include "screen.h"
#include "tty.h"
#include "snake.h"

// The characters to be shown and their colors.
char scr[80][24];
char color[80][24];

// Data variables for the game...
int px;
int py;
int dx;
int dy;
int prevdx;
int prevdy;
int money;
enum { SPLASH, RUNNING } phase = SPLASH;
int splash_ticks;
int health;

#define MAXLEN 40
struct {
  int8_t x;
  int8_t y;
} body[MAXLEN];
int bodylen;

// Print a message at screen coordinate x,y in color c.
void msg(int x, int y, int c, const char *s)
{
  int n;
  int len = strlen(s);
  for(n=0; n<len && n+x<80; n++) {
    color[x+n][y] = c;
    scr[x+n][y] = s[n];
  }
}

// Check two points, (x1,y1) and (x2,y2) and return a 1
// if they are within 5 character cells of each other.
int tooclose(int x1, int y1, int x2, int y2)
{
  int x = x1 - x2;
  int y = y1 - y2;
  if (x*x + y*y < 25)
    return 1;
  return 0;
}

// Put a dollar sign in a random location of the screen.
// Make sure that there is nothing there though.
// Also, make sure it is not too close to the snake's head.
// Let's not make the game too easy.
void newmoney(void)
{
  int x,y;
  do {
    x = (random() % 60) + 1;
    y = (random() % 22) + 1;
  } while(scr[x][y] != ' ' || tooclose(x,y, px,py));
  scr[x][y] = '$';
  color[x][y] = 0xe0;
}

// Eat some money.  Increment the money counter.
// Display the total money in the upper right of the screen.
void getmoney(void)
{
  money++;
  health++;
  char buf[30];
  sprintf(buf, "Money:%d", money);
  msg(70,0, 0xe0, buf);
  sprintf(buf, "Health:%d", health);
  msg(70,2, 0xe0, buf);
  newmoney();
}

// Indicate a collision with a big red X.
// Also display a "Game over" statement.
void collision(void)
{
  msg(px,py,0x90, "X");
  msg(px-1,py-1,0x90, "\\");
  msg(px+1,py-1,0x90, "/");
  msg(px+1,py+1,0x90, "\\");
  msg(px-1,py+1,0x90, "/");
  if (py > 12) {
    msg(30,8, 0x0f, "   Game over   ");
    msg(30,9, 0x0f, " Press any key ");
    msg(30,10,0x0f,"Enter your initials:");
    msg(30,11,0x0f,"                 ");
  } else {
    msg(30,15, 0x0f,"   Game over   ");
    msg(30,16, 0x0f," Press any key ");
    msg(30,17,0x0f,"Enter your initials:");
    msg(30,18,0x0f,"                 ");
  }

}

// Initialize the game data structures.
void init(void)
{
  int x,y;
  for(y=0; y<24; y++)
    for(x=0; x<62; x++) {
      scr[x][y] = ' ';
      color[x][y] = 0xf0;
    }

  for(x=0; x<62; x++) {
    if (x==0 || x==61) {
      msg(x,0,0xb0,"+");
      msg(x,23,0xb0,"+");
    } else {
      msg(x,0,0xb0,"-");
      msg(x,23,0xb0,"-");
    }
  }
  for(y=1; y<23; y++) {
    msg(0,y, 0xb0, "|");
    msg(61,y, 0xb0, "|");
  }

  //msg(68,5,0x0f,"length = 6");
  //msg(68,6,0x0f,"money = 0");
  px=30;
  py=12;
  scr[px][py] = '@';
  color[px][py] = 0xa0;
  bodylen = 6;
  body[0].x = px;
  body[0].y = py;
  for(x=1; x<6; x++) {
    scr[px-x][py] = '-';
    color[px-x][py] = 0xa0;
    body[x].x = px-x;
    body[x].y = py;
  }
  dx=1;
  dy=0;
  prevdx = dx;
  prevdy = dy;

  money = 0;
  health = 100;
  newmoney();
}

// Dump the scr and color arrays to the terminal screen.
void render(void)
{
  int x,y;
  home();
  int col = color[0][0];
  fgbg(col);
  for(y=0; y<24; y++) {
    setpos(0,y);
    for(x=0; x<80; x++) {
      if (color[x][y] != col) {
        col = color[x][y];
        fgbg(col);
      }
      putchar(scr[x][y]);
    }
  }
  fflush(stdout);
}

// Display the initial splash screen.
void splash(void)
{
  clear();
  int x,y;
  for(y=0; y<24; y++)
    for(x=0; x<80; x++) {
      scr[x][y] = ' ';
      color[x][y] = 0x70;
    }
  msg(30,8, 0x0a, "                 ");
  msg(30,9, 0x0a, "   Hungry Snake  ");
  msg(30,10,0x0a, "  Welcome to the ");
  msg(30,11,0x0a, "   hardest game  ");
  render();
}

// Extend the snake into the new position.
void extend(void)
{
  scr[px][py] = '@'; // draw new head
  color[px][py] = 0xa0;
  if (dx != 0) {
    if (prevdx != 0) // Did we turn a corner?
      scr[body[0].x][body[0].y] = '-'; // no
    else
      scr[body[0].x][body[0].y] = '+'; // yes
  } else {
    if (prevdy != 0) // Did we turn a corner?
      scr[body[0].x][body[0].y] = '|'; // no
    else
      scr[body[0].x][body[0].y] = '+'; // yes
  }
  int n;
  for(n=bodylen; n>=0; n--)
    body[n] = body[n-1];
  body[0].x = px;
  body[0].y = py;
  bodylen += 1;
  char buf[100];
  sprintf(buf, "Length:%d", bodylen);
  msg(70,1,0xe0,buf);
}

// Move the snake into the new position.
// And erase the last character of the tail.
void move(void)
{
    scr[body[bodylen-1].x][body[bodylen-1].y] = ' '; // erase end of tail
    scr[px][py] = '@'; // draw new head
    if (health >= 50){
        if (health==99){
             scr[79][2] = ' ';
             putchar(scr[79][2]);
             fflush(stdout);
        }
        color[px][py] = 0xa0;
    }
    else{
        if (health==9){
             scr[78][2]=' ';
             putchar(scr[78][2]);
             fflush(stdout);
        }
        color[px][py] = 0xc0;
    }
    if (dx != 0) {   //if snake moving vertically
      if (prevdx != 0) // Did we turn a corner?
        scr[body[0].x][body[0].y] = '-'; // no
      else
        scr[body[0].x][body[0].y] = '+'; // yes
    } else {          //if snake moving horizontally
      if (prevdy != 0) // Did we turn a corner?
        scr[body[0].x][body[0].y] = '|'; // no
      else
        scr[body[0].x][body[0].y] = '+'; // yes
    }
    int n;
    for(n=bodylen-1; n>0; n--) {
      body[n] = body[n-1];
    }
    body[0].x = px;
    body[0].y = py;

    char buf[100];
    sprintf(buf, "Health:%d",health);
    msg(70,2,0xe0,buf);
    render();
}

//telemove is called when the next char we're about to reach is a wall
void telemove(void){
    scr[body[bodylen-1].x][body[bodylen-1].y] = ' '; // erase end of tail
    //if it's going to hit the left wall
    if ((dx==-1)&&(px==0)){
        px = 60;
    }//else if it's going to hit the right wall
    else if ((dx==1)&&(px==61)){
        px = 1;
    }//else if it's going to hit top wall
    else if ((dy==-1)&&(py==0)){
        py = 22;
    }//if its hitting bottom wall
    else if ((dy==1)&&py==23){
        py = 1;
    }
    scr[px][py] = '@';
    if (health >= 50){
        if (health==99){
            scr[79][2] = ' ';
            putchar(scr[79][2]);
            fflush(stdout);
        }
        color[px][py] = 0xa0;
    }
    else{
        if (health==9){
            scr[78][2]=' ';
            putchar(scr[78][2]);
            fflush(stdout);
        }
        color[px][py] = 0xc0;
    }

    int n;
    for(n=bodylen-1; n>0; n--) {
       body[n] = body[n-1];
    }
    body[0].x=px;
    body[0].y=py;
}


// Interpret a key press and update the data structures.
void update(char in)
{
  prevdx = dx;
  prevdy = dy;
  switch(in) {
    case 'a':
    case 'h': dx=-1; dy=0; break;
    case 's':
    case 'j': dx=0; dy=1; break;
    case 'w':
    case 'k': dx=0; dy=-1; break;
    case 'd':
    case 'l': dx=1; dy=0; break;
    default: break;
  }

  px += dx;
  py += dy;

  if (scr[px][py] == '$') {
    getmoney();
    if (bodylen == MAXLEN-1)
      msg(26,0,0x90,"Omega-Snake-Status-Achieved");
    if (bodylen < MAXLEN){
        extend();
    }
    else
      move();
  } else if (scr[px][py] == ' ') {
      if (--health == 0){
          collision();
          phase = SPLASH;
          splash_ticks = 0;
          render();
          scr[77][2] = '0';
          putchar(scr[77][2]);
          fflush(stdout);
          return;
      }
      move();
  } else if ((scr[px][py] == '|' || scr[px][py] == '-' )&&(color[px][py]==0xb0)){
      //if it hits a wall, we don't call collision on it.
      //instead we teleport the snake to the other end of the screen
      if (--health == 0){
          collision();
          phase = SPLASH;
          splash_ticks = 0;
          char buf[100];
          sprintf(buf, "Health:%d",health);
          msg(70,2,0xe0,buf);
          render();
          //when it ends, we need to prompt for initials
          cooked_mode();
          char initials[5];
              if (py>12){
                  setpos(35,11);
              }
              else{
                  setpos(35,18);
              }
          fgets(initials,5,stdin);
          raw_mode();
          return;
      }
      telemove();
      render();
  }

  else {
    collision();
    phase = SPLASH;
    splash_ticks=0;
    render();
    //when it ends, we need to prompt for initials
    cooked_mode();
    char initials[5];
    if (py>12){
        setpos(35,11);
    }
    else{
        setpos(35,18);
    }
    fgbg(0x0f);
    fgets(initials,5,stdin);
    msg(70,8,0xd0,initials);
    render();
    raw_mode();
  }
}

void animate(void)
{
  if (phase == SPLASH) {
    if (splash_ticks < 10) {
      while (available())
        getchar();
      splash_ticks++;
      return;
    }
    // Stall waiting for a key.
    while (!available())
      ;
    getchar();
    // Get the timer counter value for the random seed.
    int seed=get_seed();
    srandom(seed);
    init();
    clear();
    phase = RUNNING;
    char buf[100];
    sprintf(buf, "Length:%d", bodylen);
    msg(70,1,0xe0,buf);
    sprintf(buf, "Money:%d",money);
    msg(70,0,0xe0,buf);
    sprintf(buf, "Health:%d",health);
    msg(70,2,0xe0,buf);
  }
  char in=' ';
  if (phase == RUNNING) {
    while(available()) {
      in = getchar();
    }
    if (in == 'q') {
#ifdef __linux__
      cursor_on();
      cooked_mode();
      exit(0);
#else
      splash();
      render();
      phase = SPLASH;
      splash_ticks = 0;
#endif
    }
    if (in == 'p') {
      freeze();
    }
    update(in);

    render();
    return;
  }
}
