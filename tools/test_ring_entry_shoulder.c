#include "ring_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t source[154U*304U],image[154U*304U];
static track_vision_result_t result;
static track_ring_vision_state_t vision;
static ring_control_state_t control;
static ring_corner_frame_t mirrored;

int main(int argc,char **argv)
{
    assert(argc==2);
    FILE *file=fopen(argv[1],"rb");
    assert(file && fread(source,1,sizeof(source),file)==sizeof(source));
    fclose(file);

    track_ring_vision_init(&vision);
    ring_control_init(&control);
    for(unsigned frame=1U;frame<=3U;++frame) {
        memcpy(image,source,sizeof(image));
        track_vision_process(image,154U,304U,&result);
        result.frame_number=frame;
        track_ring_vision_process(&vision,image,154U,304U,&result);
        const ring_corner_frame_t *corners=&result.ring.corners;
        const ring_corner_guide_t *guide=&corners->guide[ring_pdf_guide(1U,1U)];
        assert(corners->entry[1] && !corners->entry[0]);
        assert(corners->a[1].quality && corners->p[1].quality);
        assert(guide->valid && guide->error>0 && guide->confidence>=45U);
        ring_control_observe(&control,&result.ring,frame*20U,1U);
        ring_control_select_path(&control,&result.ring);
    }
    assert(control.action_locked && control.direction==-1 && control.corner_stage==1U);
    assert(control.corner_guide_id==ring_pdf_guide(1U,1U));
    assert(control.selected_path.valid && control.selected_path.error>0);

    for(unsigned y=0U;y<154U;++y)
        for(unsigned x=0U;x<304U;++x)
            image[y*304U+x]=source[y*304U+303U-x];
    ring_corner_process(image,154U,304U,0,&mirrored);
    assert(mirrored.entry[0] && !mirrored.entry[1]);
    assert(mirrored.guide[ring_pdf_guide(1U,0U)].valid);
    assert(mirrored.guide[ring_pdf_guide(1U,0U)].error<0);
    puts("PASS captured curved shoulder: right G10 entry and mirrored left G09");
    return 0;
}
