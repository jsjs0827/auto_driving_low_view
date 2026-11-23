#include "visual_motion.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
namespace perception::motion {
namespace {
using V=std::array<double,3>;using M=std::array<double,9>;
V add(const V&a,const V&b){return {{a[0]+b[0],a[1]+b[1],a[2]+b[2]}};}
V sub(const V&a,const V&b){return {{a[0]-b[0],a[1]-b[1],a[2]-b[2]}};}
V mul(const V&a,double s){return {{a[0]*s,a[1]*s,a[2]*s}};}
double dot(const V&a,const V&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
V cross(const V&a,const V&b){return {{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}};}
double norm(const V&a){return std::sqrt(dot(a,a));}
V mv(const M&r,const V&p){return {{r[0]*p[0]+r[1]*p[1]+r[2]*p[2],r[3]*p[0]+r[4]*p[1]+r[5]*p[2],r[6]*p[0]+r[7]*p[1]+r[8]*p[2]}};}
M transpose(const M&r){return {{r[0],r[3],r[6],r[1],r[4],r[7],r[2],r[5],r[8]}};}
M mm(const M&a,const M&b){M c{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)c[i*3+j]+=a[i*3+k]*b[k*3+j];return c;}
struct Pair {V previous,current;};
struct Transform {M r{{1,0,0,0,1,0,0,0,1}};V t{{0,0,0}};};
bool finitePositive(double x){return std::isfinite(x)&&x>0;}
bool calibration(const psdk_camera_intrinsics&k,int w,int h){return k.width==unsigned(w)&&k.height==unsigned(h)&&finitePositive(k.fx)&&finitePositive(k.fy)&&finitePositive(k.bxf)&&std::isfinite(k.cx)&&std::isfinite(k.cy)&&k.cx>=0&&k.cx<w&&k.cy>=0&&k.cy<h;}
bool sameCalibration(const psdk_camera_intrinsics&a,const psdk_camera_intrinsics&b){return a.width==b.width&&a.height==b.height&&a.fx==b.fx&&a.fy==b.fy&&a.cx==b.cx&&a.cy==b.cy&&a.bxf==b.bxf;}
bool imageValid(const r36::temporal::Frame&f){const auto&im=f.left.metadata;const auto required=std::uint64_t(im.stride)*im.height*3/2;
    return im.struct_size==sizeof(psdk_frame)&&im.abi_version==PSDK_ABI_VERSION&&
        im.width>=8&&im.height>=8&&im.width<=8192&&im.height<=8192&&im.stride>=im.width&& !((im.width|im.height|im.stride)&1U)&&
        (im.format==PSDK_FORMAT_NV12||im.format==PSDK_FORMAT_I420)&&f.left.storage&&f.left.storage->size()>=required&&
        f.disparity&&f.disparity->size()==std::size_t(im.width)*im.height;
}
bool point(const r36::temporal::Frame&f,double x,double y,const VisualSettings&s,V&out){
    const int w=int(f.left.metadata.width),h=int(f.left.metadata.height);
    if(x<0||y<0||x>=w-1||y>=h-1)return false;
    const int ix=int(std::floor(x)),iy=int(std::floor(y));const double ax=x-ix,ay=y-iy;
    const auto&q=*f.disparity;const unsigned q00=q[std::size_t(iy)*w+ix],q10=q[std::size_t(iy)*w+ix+1],q01=q[std::size_t(iy+1)*w+ix],q11=q[std::size_t(iy+1)*w+ix+1];
    // Bilinear interpolation never fills an invalid neighbor.
    if(!q00||!q10||!q01||!q11)return false;
    const double d=((q00*(1-ax)+q10*ax)*(1-ay)+(q01*(1-ax)+q11*ax)*ay)/256.;
    if(d<s.min_disparity_px)return false;
    const auto&k=f.intrinsics;const double z=k.bxf/(d*1000.); // bxf is pixel * mm; Q8 is pixel * 256
    if(!finitePositive(z)||z>s.max_depth_m)return false;
    out={{(x-k.cx)*z/k.fx,(y-k.cy)*z/k.fy,z}};return true;
}
double texture(const r36::temporal::Frame&f,int x,int y,int radius){
    const auto&im=f.left.metadata;const auto&p=*f.left.storage;double sum=0,sq=0;const int side=2*radius+1,n=side*side;
    for(int j=-radius;j<=radius;++j)for(int i=-radius;i<=radius;++i){const double v=p[std::size_t(y+j)*im.stride+x+i];sum+=v;sq+=v*v;}
    return std::sqrt(std::max(0.,sq/n-(sum/n)*(sum/n)));
}
double patchError(const r36::temporal::Frame&a,int ax,int ay,const r36::temporal::Frame&b,int bx,int by,int radius){
    const auto&x=*a.left.storage;const auto&y=*b.left.storage;double mean=0;const int side=radius*2+1,n=side*side;
    for(int j=-radius;j<=radius;++j)for(int i=-radius;i<=radius;++i)mean+=int(x[std::size_t(ay+j)*a.left.metadata.stride+ax+i])-int(y[std::size_t(by+j)*b.left.metadata.stride+bx+i]);
    mean/=n;double error=0;
    for(int j=-radius;j<=radius;++j)for(int i=-radius;i<=radius;++i)error+=std::abs(int(x[std::size_t(ay+j)*a.left.metadata.stride+ax+i])-int(y[std::size_t(by+j)*b.left.metadata.stride+bx+i])-mean);
    return error/n;
}
struct Pixel {int x=0,y=0;double score=0;};
bool matchPatch(const r36::temporal::Frame&a,int ax,int ay,const r36::temporal::Frame&b,int center_x,int center_y,const Roi&roi,const VisualSettings&s,Pixel&match){
    const int p=int(s.patch_radius),r=int(s.search_radius);
    const int minx=std::max(roi.x+p,center_x-r),maxx=std::min(roi.x+roi.width-p-1,center_x+r);
    const int miny=std::max(roi.y+p,center_y-r),maxy=std::min(roi.y+roi.height-p-1,center_y+r);
    if(minx>maxx||miny>maxy)return false;
    std::vector<Pixel> candidates;candidates.reserve(std::size_t(maxx-minx+1)*(maxy-miny+1));
    Pixel best;best.score=std::numeric_limits<double>::infinity();
    for(int y=miny;y<=maxy;++y)for(int x=minx;x<=maxx;++x){const double cost=patchError(a,ax,ay,b,x,y,p);candidates.push_back({x,y,cost});if(cost<best.score)best={x,y,cost};}
    double second=std::numeric_limits<double>::infinity();
    for(const auto&c:candidates)if(std::abs(c.x-best.x)>1||std::abs(c.y-best.y)>1)second=std::min(second,c.score);
    if(best.score>s.max_patch_error||!std::isfinite(second)||best.score>=s.match_ratio*second)return false;
    if(texture(b,best.x,best.y,p)<s.min_texture)return false;
    match=best;return true;
}
// Horn's absolute orientation. Jacobi diagonalization of the symmetric 4x4
// quaternion matrix avoids external linear algebra and always yields det(R)=+1.
bool fit(const std::vector<Pair>&pairs,const std::vector<std::size_t>&ids,Transform&out){
    if(ids.size()<3)return false;V p{},q{};for(auto i:ids){p=add(p,pairs[i].previous);q=add(q,pairs[i].current);}p=mul(p,1./ids.size());q=mul(q,1./ids.size());
    double s[3][3]{};double spread=0;
    for(auto id:ids){const auto a=sub(pairs[id].previous,p),b=sub(pairs[id].current,q);spread+=dot(a,a);for(int i=0;i<3;++i)for(int j=0;j<3;++j)s[i][j]+=a[i]*b[j];}
    if(spread<1e-10)return false;
    const double trace=s[0][0]+s[1][1]+s[2][2];
    double n[4][4]={{trace,s[1][2]-s[2][1],s[2][0]-s[0][2],s[0][1]-s[1][0]},
        {s[1][2]-s[2][1],s[0][0]-s[1][1]-s[2][2],s[0][1]+s[1][0],s[0][2]+s[2][0]},
        {s[2][0]-s[0][2],s[0][1]+s[1][0],-s[0][0]+s[1][1]-s[2][2],s[1][2]+s[2][1]},
        {s[0][1]-s[1][0],s[0][2]+s[2][0],s[1][2]+s[2][1],-s[0][0]-s[1][1]+s[2][2]}};
    double eigenvectors[4][4]{};for(int i=0;i<4;++i)eigenvectors[i][i]=1;
    for(int iteration=0;iteration<80;++iteration){int a=0,b=1;double largest=std::abs(n[0][1]);for(int i=0;i<4;++i)for(int j=i+1;j<4;++j)if(std::abs(n[i][j])>largest){a=i;b=j;largest=std::abs(n[i][j]);}
        if(largest<1e-12)break;const double angle=.5*std::atan2(2*n[a][b],n[b][b]-n[a][a]),c=std::cos(angle),d=std::sin(angle);
        const double aa=n[a][a],bb=n[b][b],ab=n[a][b];
        for(int k=0;k<4;++k)if(k!=a&&k!=b){const double ka=n[k][a],kb=n[k][b];n[k][a]=n[a][k]=c*ka-d*kb;n[k][b]=n[b][k]=d*ka+c*kb;}
        n[a][a]=c*c*aa-2*c*d*ab+d*d*bb;n[b][b]=d*d*aa+2*c*d*ab+c*c*bb;n[a][b]=n[b][a]=0;
        for(int k=0;k<4;++k){const double ka=eigenvectors[k][a],kb=eigenvectors[k][b];eigenvectors[k][a]=c*ka-d*kb;eigenvectors[k][b]=d*ka+c*kb;}
    }
    int index=0;for(int i=1;i<4;++i)if(n[i][i]>n[index][index])index=i;
    const double w=eigenvectors[0][index],x=eigenvectors[1][index],y=eigenvectors[2][index],z=eigenvectors[3][index];
    out.r={{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}};
    out.t=sub(q,mv(out.r,p));return true;
}
double residual(const Pair&p,const Transform&t){return norm(sub(add(mv(t.r,p.previous),t.t),p.current));}
bool noncollinear(const V&a,const V&b,const V&c){return norm(cross(sub(b,a),sub(c,a)))>1e-6;}
std::vector<std::size_t> inliers(const std::vector<Pair>&pairs,const Transform&t,double threshold){std::vector<std::size_t> result;for(std::size_t i=0;i<pairs.size();++i)if(residual(pairs[i],t)<=threshold)result.push_back(i);return result;}
}
VisualMotion::VisualMotion(VisualSettings s):settings_(s){
    const double positives[]={s.min_texture,s.max_patch_error,s.match_ratio,s.min_disparity_px,s.max_depth_m,s.inlier_threshold_m,s.max_residual_m,s.min_inlier_ratio,s.min_dt_ms,s.max_gap_ms,s.max_translation_m,s.max_rotation_rad};
    for(double v:positives)if(!finitePositive(v))throw std::invalid_argument("Invalid visual motion numeric setting");
    if(s.patch_radius<1||s.patch_radius>8||s.search_radius<1||s.search_radius>64||s.grid_step<2||s.grid_step>512||s.max_features<3||s.max_features>5000||s.min_matches<3||s.min_matches>s.max_features||s.min_inliers<3||s.min_inliers>s.min_matches||s.ransac_iterations<1||s.ransac_iterations>10000||s.match_ratio>=1||s.min_inlier_ratio>1||s.min_dt_ms>s.max_gap_ms||s.roi.x<0||s.roi.y<0||s.roi.width<0||s.roi.height<0||(bool(s.roi.width)!=bool(s.roi.height)))throw std::invalid_argument("Invalid visual motion ranges");
    for(double v:s.extrinsics.rotation)if(!std::isfinite(v))throw std::invalid_argument("Nonfinite camera/body rotation");
    for(double v:s.extrinsics.translation_m)if(!std::isfinite(v))throw std::invalid_argument("Nonfinite camera/body translation");
    const auto identity=mm(s.extrinsics.rotation,transpose(s.extrinsics.rotation));for(int i=0;i<9;++i)if(std::abs(identity[i]-(i%4==0?1.:0.))>1e-5)throw std::invalid_argument("Camera/body rotation must be orthonormal");
    const auto&r=s.extrinsics.rotation;const double det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);if(std::abs(det-1)>1e-5)throw std::invalid_argument("Camera/body rotation must have determinant +1");
}
VisualResult VisualMotion::estimate(const r36::temporal::Window&w)const {
    VisualResult out;out.body_confirmed=settings_.extrinsics.confirmed;
    const auto fail=[&](const char* r){out.reason=r;return out;};
    if(!w.sampled)return fail("current_capture_not_sampled");
    if(w.frames.size()<2)return fail("insufficient_history");
    if(!w.frames[w.frames.size()-2]||!w.frames.back())return fail("invalid_history_frame");
    const auto&a=*w.frames[w.frames.size()-2];const auto&b=*w.frames.back();
    out.t0_ns=a.left.metadata.timestamp_ns;out.t1_ns=b.left.metadata.timestamp_ns;
    if(!out.t0_ns||out.t1_ns<=out.t0_ns)return fail("invalid_capture_timestamps");
    out.dt_s=double(out.t1_ns-out.t0_ns)*1e-9;
    if(out.dt_s*1000<settings_.min_dt_ms||out.dt_s*1000>settings_.max_gap_ms)return fail("capture_time_gap");
    if(!imageValid(a)||!imageValid(b))return fail("missing_or_invalid_image_disparity");
    const int width=int(a.left.metadata.width),height=int(a.left.metadata.height);
    if(b.left.metadata.width!=a.left.metadata.width||b.left.metadata.height!=a.left.metadata.height||!calibration(a.intrinsics,width,height)||!calibration(b.intrinsics,width,height)||!sameCalibration(a.intrinsics,b.intrinsics))return fail("calibration_or_geometry_changed");
    Roi roi=settings_.roi;if(!roi.width&&!roi.height)roi={0,0,width,height};
    if(roi.x>=width||roi.y>=height||roi.width>width-roi.x||roi.height>height-roi.y||roi.width<int(2*settings_.patch_radius+3)||roi.height<int(2*settings_.patch_radius+3))return fail("invalid_visual_roi");
    const int p=int(settings_.patch_radius);std::vector<Pixel> candidates;
    for(int y=roi.y+p;y<roi.y+roi.height-p-1;y+=int(settings_.grid_step))for(int x=roi.x+p;x<roi.x+roi.width-p-1;x+=int(settings_.grid_step)){
        V point3;if(!point(a,x,y,settings_,point3))continue;const double score=texture(a,x,y,p);if(score>=settings_.min_texture)candidates.push_back({x,y,score});}
    std::stable_sort(candidates.begin(),candidates.end(),[](const Pixel&a,const Pixel&b){return a.score>b.score;});
    if(candidates.size()>settings_.max_features)candidates.resize(settings_.max_features);out.features=static_cast<unsigned>(candidates.size());
    if(candidates.size()<settings_.min_matches)return fail("insufficient_textured_features");
    std::vector<Pair> pairs;
    for(const auto&c:candidates){Pixel target,back;
        if(!matchPatch(a,c.x,c.y,b,c.x,c.y,roi,settings_,target)||!matchPatch(b,target.x,target.y,a,c.x,c.y,roi,settings_,back)||std::abs(back.x-c.x)>1||std::abs(back.y-c.y)>1)continue;
        V previous,current;if(point(a,c.x,c.y,settings_,previous)&&point(b,target.x,target.y,settings_,current))pairs.push_back({previous,current});
    }
    out.matches=static_cast<unsigned>(pairs.size());if(pairs.size()<settings_.min_matches)return fail("insufficient_patch_matches");
    std::vector<std::size_t> best;Transform transform;std::uint64_t rng=0x7261667473746572ULL;
    const auto random=[&](){rng^=rng<<13;rng^=rng>>7;rng^=rng<<17;return std::size_t(rng%pairs.size());};
    double best_error=std::numeric_limits<double>::infinity();
    for(unsigned iteration=0;iteration<settings_.ransac_iterations;++iteration){const auto i=random(),j=random(),k=random();
        if(i==j||i==k||j==k||!noncollinear(pairs[i].previous,pairs[j].previous,pairs[k].previous)||!noncollinear(pairs[i].current,pairs[j].current,pairs[k].current))continue;
        Transform hypothesis;if(!fit(pairs,{i,j,k},hypothesis))continue;auto good=inliers(pairs,hypothesis,settings_.inlier_threshold_m);double error=0;for(auto id:good)error+=residual(pairs[id],hypothesis);
        if(good.size()>best.size()||(good.size()==best.size()&&error<best_error)){best=std::move(good);transform=hypothesis;best_error=error;}
    }
    out.inliers=static_cast<unsigned>(best.size());
    if(best.size()<settings_.min_inliers||double(best.size())/pairs.size()<settings_.min_inlier_ratio)return fail("insufficient_rigid_motion_inliers");
    if(!fit(pairs,best,transform))return fail("degenerate_rigid_motion");
    best=inliers(pairs,transform,settings_.inlier_threshold_m);out.inliers=static_cast<unsigned>(best.size());
    if(best.size()<settings_.min_inliers||double(best.size())/pairs.size()<settings_.min_inlier_ratio)return fail("insufficient_refined_inliers");
    if(!fit(pairs,best,transform))return fail("degenerate_refined_motion");
    double squared=0;for(auto id:best){const double r=residual(pairs[id],transform);squared+=r*r;}
    out.residual_m=std::sqrt(squared/best.size());if(out.residual_m>settings_.max_residual_m)return fail("rigid_motion_residual_too_large");
    const double angle=std::acos(std::clamp((transform.r[0]+transform.r[4]+transform.r[8]-1)*.5,-1.,1.));
    if(norm(transform.t)>settings_.max_translation_m||angle>settings_.max_rotation_rad)return fail("implausible_visual_motion");
    out.scene_rotation=transform.r;out.scene_translation_m=transform.t;
    const auto&e=settings_.extrinsics;
    // Conjugation by T_body_camera accounts for both axis rotation and lever arm:
    // T_B1_B0 = T_B_C * T_C1_C0 * inverse(T_B_C); then invert for body displacement.
    const M body_scene_r=mm(mm(e.rotation,transform.r),transpose(e.rotation));
    const V body_scene_t=sub(add(mv(e.rotation,transform.t),e.translation_m),mv(body_scene_r,e.translation_m));
    out.body_rotation=transpose(body_scene_r);out.body_translation_m=mul(mv(out.body_rotation,body_scene_t),-1);
    out.forward_m=out.body_translation_m[0];out.lateral_m=out.body_translation_m[1];out.yaw_rad=std::atan2(out.body_rotation[3],out.body_rotation[0]);
    out.quality=double(best.size())/pairs.size()*std::exp(-out.residual_m/settings_.max_residual_m);
    out.available=true;out.reason=out.body_confirmed?"ok":"camera_motion_only_extrinsics_unconfirmed";return out;
}
}
