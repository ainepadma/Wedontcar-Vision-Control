#include "ring_corner.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

enum { HEIGHT=154, WIDTH=304 };
static uint8_t im[HEIGHT*WIDTH],saved[HEIGHT*WIDTH];
static ring_corner_frame_t frame;

static void check_exit(unsigned side,unsigned last_border_row,int isolated_row,int expected_end)
{
    for(unsigned y=0;y<HEIGHT;++y) for(unsigned x=0;x<WIDTH;++x) {
        int lower_border=(y>=110U && (y<=last_border_row || (int)y==isolated_row) &&
                          x<=40U+(y-110U)/4U);
        int black=(y<=60U && x<=100U) || x>=261U || lower_border;
        unsigned col=side ? WIDTH-1U-x : x;
        im[y*WIDTH+col]=(uint8_t)(black ? 0 : 255);
    }
    memcpy(saved,im,sizeof(im));
    ring_corner_process(im,HEIGHT,WIDTH,0,&frame);
    assert(!memcmp(saved,im,sizeof(im)));
    assert(frame.exit_v[side].quality && frame.exit_v[side].y>=58 && frame.exit_v[side].y<=61);
    const ring_corner_guide_t *g=&frame.guide[ring_pdf_guide(7,side)];
    if(expected_end<0) {
        assert(!g->valid && !g->boundary[side].valid);
        return;
    }
    const ring_corner_line_t *line=&g->boundary[side];
    unsigned row=expected_end==HEIGHT-1 ? HEIGHT-2U : (unsigned)expected_end;
    assert(g->valid && line->valid && line->y1==expected_end);
    assert(line->x0==frame.exit_v[side].x && line->y0==frame.exit_v[side].y);
    assert((frame.border_flags[row]&(1U<<side)) && line->x1==frame.border[side][row]);
}
int main(void)
{
    for(unsigned side=0;side<2;++side) {
        check_exit(side,153,-1,153); /* Lowest real sample, original bottom connection. */
        check_exit(side,151,-1,151); /* One missing sample. */
        check_exit(side,149,151,149); /* Skip an isolated row, use the nearby continuous edge. */
        check_exit(side,139,151,-1);  /* Far or isolated borders cannot anchor an exit. */
    }
    puts("PASS mirrored exit tips, normal and clipped bottom anchors, rejected distant/isolated rows");
    return 0;
}
