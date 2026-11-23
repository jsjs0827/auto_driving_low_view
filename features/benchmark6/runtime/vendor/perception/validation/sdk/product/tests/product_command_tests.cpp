#include "product/command_contract.hpp"
#include <iostream>
#include <stdexcept>

int main() {
    using r30::product::commandPlan;
    const auto check=[](bool ok,const char* message){if(!ok)throw std::runtime_error(message);};
    const auto map=commandPlan(1,false);
    check(map.map&&!map.station&&map.camera==1&&map.app==2001,"map protocol changed");
    const auto mark=commandPlan(2,true);
    check(!mark.map&&mark.station&&mark.camera==2&&mark.app==2002,"mark protocol changed");
    const auto both=commandPlan(3,true);
    check(both.map&&both.station&&both.camera==3&&both.app==2002,"combined protocol changed");
    const auto stop=commandPlan(21,false);
    check(!stop.map&&!stop.station&&stop.camera==4&&stop.app==2021,"stop protocol changed");
    bool map_state=true,station_state=false,camera_sent=false;
    for(const int cmd:{2,3}) {
        try {
            const auto plan=commandPlan(cmd,false);
            map_state=plan.map;station_state=plan.station;camera_sent=true;
            throw std::runtime_error("unavailable recharge accepted");
        } catch(const r28::Failure& error) {check(error.status().code==2004,"wrong recharge code");}
        check(map_state&&!station_state&&!camera_sent,"rejected command changed state");
    }
    try {commandPlan(99,true);throw std::runtime_error("unknown command accepted");}
    catch(const r28::Failure& error) {check(error.status().code==2001,"wrong unknown-command code");}
    for(const bool worker_was_running:{false,true}) {
        bool map=true,station=false,worker_running=worker_was_running;
        bool started_here=false,stopped_here=false;
        try {
            r30::product::commandTransaction([&] {
                worker_running=true;started_here=!worker_was_running;
                map=false;station=true;
                r28::Status transport;transport.code=2003;transport.native_code=-37;
                transport.detail="camera/application request failed";throw r28::Failure(transport);
            },[&] {
                map=true;station=false;
                if(started_here) {worker_running=false;stopped_here=true;}
                throw std::runtime_error("cleanup also failed");
            });
            throw std::runtime_error("transport failure was swallowed");
        } catch(const r28::Failure& error) {
            check(error.status().code==2003&&error.status().native_code==-37,"rollback replaced original failure");
        }
        check(map&&!station,"rollback did not restore previous modes");
        check(worker_running==worker_was_running&&stopped_here!=worker_was_running,"rollback stopped the wrong worker");
    }
    for(const int cmd:{1,21}) {
        const auto plan=commandPlan(cmd,true);
        bool map=plan.map,station=false,worker_running=false;
        try {
            r30::product::commandTransaction([] {
                r28::Status transport;transport.code=2003;transport.native_code=-18;
                throw r28::Failure(transport);
            },[&] {map=plan.map;station=false;worker_running=false;});
        } catch(const r28::Failure& error) {
            check(error.status().code==2003&&error.status().native_code==-18,"stop replaced transport failure");
        }
        check(map==plan.map&&!station&&!worker_running,"failed stop restarted previous worker");
    }
    std::cout<<"original commands, rejection, startup rollback and stop failure contracts passed\n";
}
