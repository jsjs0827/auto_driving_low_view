#pragma once
namespace demo_camera {
// Shared by the real SDK adapter and the host failure/ordering tests. A failed
// close must never unload code that vendor threads could still be executing.
template<class Restore,class Close,class Log>
bool closeWithPolicy(bool restore_before_close, Restore restore, Close close, Log log) {
    if(restore_before_close) {const int code=restore();log("restore_mode",code);}
    const int code=close();log("moCloseCamera",code);return code==0;
}
}
