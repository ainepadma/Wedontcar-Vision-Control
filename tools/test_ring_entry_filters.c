/* One translation unit intentionally exercises the actual private component
 * validator as well as public image detection. Do not link ring_corner.c twice. */
#include "../demo/user/src/ring_corner.c"
#include <assert.h>
#include <stdio.h>

static uint8_t test_image[154U*304U],test_saved[154U*304U];
static ring_corner_frame_t test_frame;

static void block(unsigned x,unsigned y,unsigned width,unsigned height)
{
    memset(test_image,255,sizeof(test_image));
    for(unsigned row=y;row<y+height;++row)
        memset(test_image+row*304U+x,0,width);
}
static int supported(unsigned side,float x,float y)
{
    ring_corner_point_t p=point(x,y,RING_CORNER_NONE);
    return island_point(test_image,154,304,side,&p);
}
static void load_fixture(const char *directory,const char *name)
{
    char path[1024];snprintf(path,sizeof(path),"%s/%s.bin",directory,name);
    FILE *fp=fopen(path,"rb");assert(fp);
    assert(fread(test_image,1,sizeof(test_image),fp)==sizeof(test_image));fclose(fp);
}
int main(int argc,char **argv)
{
    assert(argc==3);
    int expected_entry=atoi(argv[2]);
    /* Two equal runs around the anchor must leave the row unobserved. */
    memset(test_image,0,sizeof(test_image));
    for(unsigned y=0;y<154;++y) {
        memset(test_image+y*304+40,255,60);
        memset(test_image+y*304+204,255,60);
    }
    ring_corner_process(test_image,154,304,0,&test_frame);
    assert(flags[100]==0 && b[0][100]==0 && b[1][100]==303);
    block(40,30,40,30);assert(supported(0,80,45));
    block(0,30,40,30);assert(supported(0,40,45)==!RING_ENTRY_REQUIRE_CLOSED_ISLAND);
    block(264,30,40,30);assert(supported(1,263,45)==!RING_ENTRY_REQUIRE_CLOSED_ISLAND);
    block(40,0,40,30);assert(supported(0,80,15)==!RING_ENTRY_REQUIRE_CLOSED_ISLAND);
    block(40,124,40,30);assert(supported(0,80,140)==!RING_ENTRY_REQUIRE_CLOSED_ISLAND);
    block(0,0,40,30);assert(supported(0,40,15)==!RING_ENTRY_REQUIRE_CLOSED_ISLAND);
    block(40,30,4,4);assert(!supported(0,44,31));
    block(40,35,100,2);assert(!supported(0,140,35));
    block(40,20,128,64);assert(supported(0,168,40)); /* exactly 8192 */
    block(40,20,100,100);assert(!supported(0,140,70));
    puts("PASS edge/corner contact switch, tiny/thin noise, exact capacity and oversized background");

    load_fixture(argv[1],"entry");memcpy(test_saved,test_image,sizeof(test_image));
    ring_corner_process(test_image,154,304,0,&test_frame);
    assert(!memcmp(test_saved,test_image,sizeof(test_image)));
    assert(test_frame.entry[0]==expected_entry && !test_frame.entry[1]);
    for(unsigned y=0;y<154;++y) for(unsigned x=0;x<304;++x)
        test_image[y*304U+x]=test_saved[y*304U+303U-x];
    ring_corner_process(test_image,154,304,0,&test_frame);
    assert(test_frame.entry[1]==expected_entry && !test_frame.entry[0]);
    puts("PASS original/mirrored entry, configured A/P range and immutable input");

    const char *ordinary[]={"ordinary_p1","ordinary_p2","ordinary_p9","ordinary_p10",
        "ordinary_p1_alternate","ordinary_p2_alternate","ordinary_p9_alternate","ordinary_p10_alternate"};
    for(unsigned i=0;i<8;++i) {
        load_fixture(argv[1],ordinary[i]);ring_corner_process(test_image,154,304,0,&test_frame);
        assert(!test_frame.entry[0] && !test_frame.entry[1]);
    }
    puts("PASS eight ordinary-road images rejected as 360 entries");
    unsigned cases=0;
    for(unsigned i=0;i<9;++i) {
        load_fixture(argv[1],i<8 ? ordinary[i] : "entry");
        memcpy(test_saved,test_image,sizeof(test_image));
        for(int dy=-2;dy<=2;++dy) for(int dx=-2;dx<=2;++dx) for(unsigned mirror=0;mirror<2;++mirror) {
            for(int y=0;y<154;++y) for(int x=0;x<304;++x) {
                int sx=x-dx,sy=y-dy;
                if(sx<0) sx=0;
                if(sx>303) sx=303;
                if(sy<0) sy=0;
                if(sy>153) sy=153;
                if(mirror) sx=303-sx;
                test_image[y*304+x]=test_saved[sy*304+sx];
            }
            ring_corner_process(test_image,154,304,0,&test_frame);
            int expected=i==8 ? expected_entry : 0;
            if(test_frame.entry[mirror]!=expected || test_frame.entry[1-mirror]) {
                fprintf(stderr,"Unexpected entry: image=%u shift=%d,%d mirror=%u got=%u,%u\n",
                    i,dx,dy,mirror,test_frame.entry[0],test_frame.entry[1]);
                return 1;
            }
            ++cases;
        }
    }
    printf("PASS %u translated/mirrored entry and ordinary-road cases\n",cases);return 0;
}
