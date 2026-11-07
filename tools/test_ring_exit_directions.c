#include "ring_corner.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t im[154*304],mirrored[154*304],saved[154*304];
static ring_corner_frame_t f;
int main(void){for(unsigned side=0;side<2;++side){
 memset(im,255,sizeof(im));
 for(unsigned y=0;y<154;++y)for(unsigned x=0;x<304;++x)
  if((y<=60&&x<=100)||x>=261||(y>=110&&x<=40+(y-110)/4))im[y*304+x]=0;
 for(unsigned y=0;y<154;++y)for(unsigned x=0;x<304;++x)mirrored[y*304+x]=im[y*304+(side?303-x:x)];
 memcpy(saved,mirrored,sizeof(saved));ring_corner_process(mirrored,154,304,0,&f);
 assert(!memcmp(saved,mirrored,sizeof(saved)));
 const ring_corner_guide_t*g=&f.guide[ring_pdf_guide(7,side)];
 assert(f.exit_v[side].quality&&g->valid&&g->boundary[side].valid);
 assert(f.exit_v[side].y>=58&&f.exit_v[side].y<=61);
 assert(g->boundary[side].y1==153);
 }puts("PASS mirrored axis-aligned right-angle exit, bottom connection, immutable image");return 0;}
