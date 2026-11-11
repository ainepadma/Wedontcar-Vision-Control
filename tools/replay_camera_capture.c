#include "img_process.h"
#include "ring_control.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define WIDTH 304U
#define HEIGHT 154U
#define PIXELS (WIDTH * HEIGHT)

static uint8_t raw[PIXELS], binary[PIXELS], scratch[WIDTH * 5U];
static uint32_t histogram[256];
static image_track_preprocess_state_t preprocess;
static track_ring_vision_state_t vision;
static track_vision_result_t result;
static ring_control_state_t control;

int main(int argc, char **argv)
{
    if(argc < 2) {
        fprintf(stderr, "usage: replay_camera_capture frame.raw [...frame.raw]\n");
        return 2;
    }
    ring_control_init(&control);
    track_ring_vision_init(&vision);
    unsigned frame=0U;
    for(int file_index=1;file_index<argc;++file_index) {
        FILE *input=fopen(argv[file_index],"rb");
        if(!input || fread(raw,1,sizeof(raw),input)!=sizeof(raw)) {
            fprintf(stderr,"cannot read 304x154 frame: %s\n",argv[file_index]);
            if(input) fclose(input);
            return 1;
        }
        fclose(input);
        for(unsigned repeat=0U;repeat<3U;++repeat) {
            memcpy(binary,raw,sizeof(raw));
            uint8_t threshold=255U;
            if(!image_track_preprocess_stable(binary,HEIGHT,WIDTH,0U,HEIGHT,
                scratch,sizeof(scratch),histogram,&threshold,&preprocess,++frame*20U)) return 1;
            track_vision_process(binary,HEIGHT,WIDTH,&result);
            result.frame_number=frame;
            track_ring_vision_process(&vision,binary,HEIGHT,WIDTH,&result);
            ring_control_observe(&control,&result.ring,frame*20U,1U);
            ring_control_select_path(&control,&result.ring);
            if(repeat!=2U) continue;
            const ring_corner_frame_t *corners=&result.ring.corners;
            const ring_corner_guide_t *left=&corners->guide[ring_pdf_guide(1U,0U)];
            const ring_corner_guide_t *right=&corners->guide[ring_pdf_guide(1U,1U)];
            const ring_corner_guide_t *third=&corners->guide[ring_pdf_guide(5U,1U)];
            printf("%s threshold=%u normal=%d/%d valid=%u entry=%u/%u A=%u/%u(%d,%d) P=%u/%u(%d,%d) gap=%u/%u "
                   "G09=%u:%d/%d G10=%u:%d/%d G18=%u:%d/%d stage=%u direction=%d selected=G%02u:%d/%d\n",
                argv[file_index],threshold,result.error,result.heading,result.valid,
                corners->entry[0],corners->entry[1],
                corners->a[0].quality,corners->a[1].quality,
                (int)corners->a[1].x,(int)corners->a[1].y,
                corners->p[0].quality,corners->p[1].quality,
                (int)corners->p[1].x,(int)corners->p[1].y,
                corners->gap[0],corners->gap[1],
                left->valid,left->error,left->heading,
                right->valid,right->error,right->heading,
                third->valid,third->error,third->heading,
                control.corner_stage,control.direction,control.corner_guide_id,
                control.selected_path.error,control.selected_path.heading);
            for(unsigned y=20U;y<=120U;y+=20U)
                printf("  y=%u left=%u right=%u flags=%u\n",y,corners->border[0][y],
                    corners->border[1][y],corners->border_flags[y]);
        }
        char output[1024];
        if(snprintf(output,sizeof(output),"%s.binary",argv[file_index])>=(int)sizeof(output)) return 1;
        FILE *out=fopen(output,"wb");
        if(!out || fwrite(binary,1,sizeof(binary),out)!=sizeof(binary)) {
            if(out) fclose(out);
            return 1;
        }
        fclose(out);
    }
    return 0;
}
