#include "motion_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static motion_sample_t sample(uint32_t seq)
{
    motion_sample_t s = {.boot_id=7, .generation=1, .sequence=seq,
        .interval_ms=100, .acquired_us=(int64_t)seq*100000, .xyz={1,2,3}, .valid=true};
    return s;
}
int main(void)
{
    motion_window_t w = {0};
    motion_status_t status;
    for (unsigned i=0; i<20; ++i) {
        motion_sample_t s=sample(i);
        assert(motion_window_push(&w, &s, &status) == (i==19));
    }
    motion_window_advance(&w);
    assert(w.count==10 && w.samples[0].sequence==10);
    for (unsigned i=20; i<30; ++i) {
        motion_sample_t s=sample(i);
        assert(motion_window_push(&w, &s, &status) == (i==29));
    }
    assert(w.samples[0].sequence==10 && w.samples[19].sequence==29);
    // A forgotten advance cannot overrun memory.
    motion_sample_t s=sample(30);
    assert(!motion_window_push(&w,&s,&status) && w.count==1);
    // Every admission-breaking condition must start a new window or reject.
    for(unsigned fault=0; fault<9; ++fault) {
        motion_window_reset(&w);
        s=sample(10); assert(!motion_window_push(&w,&s,&status));
        s=sample(11);
        switch(fault) {
        case 0: s.sequence++; break;
        case 1: s.sequence--; break;
        case 2: s.acquired_us+=50001; break;
        case 3: s.acquired_us-=100000; break;
        case 4: s.boot_id++; break;
        case 5: s.generation++; break;
        case 6: s.valid=false; break;
        case 7: s.xyz[1]=NAN; break;
        case 8: s.interval_ms=500; break;
        }
        assert(!motion_window_push(&w,&s,&status));
        assert(w.count==(fault<6 ? 1u : 0u));
        assert(status==(fault<6 ? MOTION_DISCONTINUITY : fault==8 ? MOTION_WAIT_100MS : MOTION_INVALID_SAMPLE));
    }
    motion_window_reset(&w);
    s=sample(UINT32_MAX); s.acquired_us=100000;
    assert(!motion_window_push(&w,&s,&status));
    s.sequence=0; s.acquired_us=250000;
    assert(!motion_window_push(&w,&s,&status) && w.count==2); // 150 ms inclusive, sequence wrap.
    s.sequence=1; s.acquired_us=400001;
    assert(!motion_window_push(&w,&s,&status) && w.count==1);
    float raw[20][3], features[60], logits[2]; int8_t q[60]; uint32_t clipped;
    for(unsigned i=0;i<20;++i) for(unsigned a=0;a<3;++a) raw[i][a]=(float)a;
    assert(motion_preprocess(raw,features));
    for(unsigned i=0;i<60;++i) assert(features[i]==0);
    features[0]=-2.5f; features[1]=-1.5f; features[2]=0.5f; features[3]=1.5f; features[4]=2.5f;
    features[5]=-129; features[6]=128;
    assert(motion_quantize(features,1,0,q,&clipped));
    assert(q[0]==-2 && q[1]==-2 && q[2]==0 && q[3]==2 && q[4]==2);
    assert(q[5]==-128 && q[6]==127 && clipped==2);
    assert(!motion_quantize(features,0,0,q,&clipped));
    features[0]=INFINITY; assert(!motion_quantize(features,1,0,q,&clipped));
    raw[0][0]=NAN; assert(!motion_preprocess(raw,features));
    int8_t output[2]={-12,-12}; assert(motion_decode(output,0.5f,-12,logits)==0);
    output[1]=0; assert(motion_decode(output,0.5f,-12,logits)==1 && logits[1]==6);
    puts("Window admission, stride, bounds, wrap, quantization ties/clipping and logits: PASS");
}
