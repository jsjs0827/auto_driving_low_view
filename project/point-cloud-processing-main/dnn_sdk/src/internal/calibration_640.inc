        // r26 original RGBD map geometry: a black first frame plus the generic
        // probes can leave modes 3 and 4 indistinguishable. Exercise
        // half-integer rounding with the unchanged reference callback before
        // caching a mode. These fixtures run only during initialization.
        if(sw==640 && sh==544 && g.valid_w==564 && g.valid_h==480){
            // Y and UV have the same source/target ratios here; identical
            // axis coordinates exercise rounding in all three channels.
            const auto plane=std::size_t(sw)*sh;
            // Destination chroma (23,127): wx=0.16666603088378906, wy=0.
            // Separate horizontal sums give 91.5; FMA gives 91.49999237.
            std::fill(fixture.begin(),fixture.end(),uint8_t(127));
            fixture[144*sw+26]=90;fixture[144*sw+27]=99;
            for(int c=0;c<2;++c){
                fixture[plane+144*sw+26*2+c]=90;
                fixture[plane+144*sw+27*2+c]=99;
            }
            if(!reference_(fixture.data(),sw,sh,sw,fixture.size(),true,dw,dh,resize,&baseline_geometry,baseline,127))return false;
            intersect({fixture.data(),fixture.size(),sw});
            // Destination chroma (34,44): separate the second-product FMA
            // family (2/5/8) from modes whose horizontal second product rounds.
            // The reference byte is 124 or 125 at this half-integer boundary.
            std::fill(fixture.begin(),fixture.end(),uint8_t(127));
            fixture[49*sw+38]=129;fixture[49*sw+39]=134;
            fixture[50*sw+38]=122;fixture[50*sw+39]=125;
            for(int c=0;c<2;++c){
                fixture[plane+49*sw+38*2+c]=129;
                fixture[plane+49*sw+39*2+c]=134;
                fixture[plane+50*sw+38*2+c]=122;
                fixture[plane+50*sw+39*2+c]=125;
            }
            if(!reference_(fixture.data(),sw,sh,sw,fixture.size(),true,dw,dh,resize,&baseline_geometry,baseline,127))return false;
            intersect({fixture.data(),fixture.size(),sw});
            // Destination chroma (23,151): include nonzero vertical weight
            // to distinguish horizontal and vertical contraction jointly.
            std::fill(fixture.begin(),fixture.end(),uint8_t(127));
            fixture[171*sw+26]=121;fixture[171*sw+27]=131;
            fixture[172*sw+26]=122;fixture[172*sw+27]=121;
            for(int c=0;c<2;++c){
                fixture[plane+171*sw+26*2+c]=121;
                fixture[plane+171*sw+27*2+c]=131;
                fixture[plane+172*sw+26*2+c]=122;
                fixture[plane+172*sw+27*2+c]=121;
            }
            if(!reference_(fixture.data(),sw,sh,sw,fixture.size(),true,dw,dh,resize,&baseline_geometry,baseline,127))return false;
            intersect({fixture.data(),fixture.size(),sw});
        }
