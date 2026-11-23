#include "background_drain.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <streambuf>
#include <thread>

namespace {
void require(bool value,const char* message) {
    if(!value)throw std::runtime_error(message);
}
std::string csvCell(const std::string& line,unsigned index) {
    std::istringstream input(line);std::string cell;
    for(unsigned i=0;i<=index;++i)if(!std::getline(input,cell,','))throw std::runtime_error("Missing CSV column");
    return cell;
}
template<class Predicate> void until(Predicate predicate,const char* message) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(!predicate()) {
        if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
template<class Action> void throwsContaining(Action action,const char* wanted) {
    bool caught=false;
    try {action();}
    catch(const std::exception& e) {caught=std::string(e.what()).find(wanted)!=std::string::npos;}
    require(caught,"Expected exception was not propagated");
}

struct State {
    std::atomic<unsigned> calls{0},completed{0},active{0};
    std::atomic<bool> destroyed{false},blocked{false},release{false};
    std::mutex mutex;
    std::condition_variable cv;
};
enum class Behavior { Normal, Throw, BadSize, NoImage, MissingMetadata, MissingTimestamp, Block };
class FakeSource final:public demo_camera::Source {
    std::shared_ptr<State> state_;
    std::shared_ptr<example::Image> image_;
    Behavior behavior_;
public:
    FakeSource(std::shared_ptr<State> state,Behavior behavior=Behavior::Normal)
        :state_(std::move(state)),image_(std::make_shared<example::Image>()),behavior_(behavior) {
        image_->width=behavior==Behavior::BadSize?1280:640;
        image_->height=544;image_->stride=std::size_t(image_->width);
        image_->format=perception::PixelFormat::NV12;
        image_->bytes.assign(image_->stride*544*3/2,127);
    }
    ~FakeSource() override {state_->destroyed.store(true);}
    const char* name() const noexcept override {return "background-drain-test";}
    demo_camera::Frame next() override {
        struct Active {
            State& state;
            explicit Active(State& s):state(s){++state.active;}
            ~Active(){--state.active;}
        } active(*state_);
        const auto id=state_->calls.fetch_add(1);
        if(behavior_==Behavior::Throw)throw std::runtime_error("source failure sentinel");
        if(behavior_==Behavior::Block) {
            std::unique_lock<std::mutex> lock(state_->mutex);
            state_->blocked.store(true);
            state_->cv.wait(lock,[&]{return state_->release.load();});
        } else std::this_thread::sleep_for(std::chrono::milliseconds(1));
        demo_camera::Frame frame;frame.info.id=id;
        frame.info.timestamp_ns=(std::uint64_t(id)+1)*30000000;
        if(behavior_!=Behavior::NoImage)frame.image=image_;
        if(behavior_!=Behavior::MissingMetadata) {
            frame.acquisition.sdk_timestamp=true;
            frame.sdk_rgbd_triplet_available=true;frame.sdk_rgbd_triplet_valid=true;
            frame.sdk_color_pair_valid=true;
            frame.sdk_right_frame_id=id;frame.sdk_disparity_frame_id=id;
            frame.sdk_right_timestamp_ns=frame.info.timestamp_ns;
            frame.sdk_disparity_timestamp_ns=frame.info.timestamp_ns;
            if(behavior_==Behavior::MissingTimestamp)frame.sdk_right_timestamp_ns=0;
        }
        ++state_->completed;
        return frame;
    }
};

class FaultBuffer final:public std::streambuf {
public:
    std::atomic<bool> fail_writes{false},fail_sync{false};
    std::string data;
protected:
    std::streamsize xsputn(const char* source,std::streamsize count) override {
        if(fail_writes.load())return 0;
        data.append(source,std::size_t(count));return count;
    }
    int_type overflow(int_type c) override {
        if(fail_writes.load())return traits_type::eof();
        if(!traits_type::eq_int_type(c,traits_type::eof()))data+=traits_type::to_char_type(c);
        return traits_type::not_eof(c);
    }
    int sync() override {return fail_sync.load()?-1:0;}
};
class FaultStream final:public std::ostream {
public:
    FaultBuffer buffer;
    FaultStream():std::ostream(nullptr){rdbuf(&buffer);}
};

void normalConsumption() {
    auto state=std::make_shared<State>();auto source=std::make_shared<FakeSource>(state);
    auto csv=std::make_shared<std::ostringstream>();
    r27::BackgroundDrain drain(source,csv);
    throwsContaining([&]{(void)drain.count();},"stop/join");
    throwsContaining([&]{(void)drain.metadataVerifiedCount();},"stop/join");
    until([&]{return state->completed.load()>=4;},"Source did not run");
    drain.check();drain.stop();drain.check();drain.stop();drain.check();
    require(drain.count()==state->completed.load() && drain.count()>=4,"Drain lost valid source frames");
    require(drain.metadataVerifiedCount()==drain.count(),"Valid metadata was not counted");
    require(drain.metadataUnverifiedCount()==0,"Valid metadata was marked missing");
    require(state->active.load()==0,"Stop returned before acquisition joined");
    const auto calls=state->calls.load();std::this_thread::sleep_for(std::chrono::milliseconds(4));
    require(state->calls.load()==calls,"Source kept acquiring after stop");
    std::istringstream input(csv->str());std::string line;std::uint64_t rows=0;
    std::getline(input,line);require(line.find("drain_triplet_verified")!=std::string::npos,"Missing metadata verification column");
    while(std::getline(input,line)){++rows;require(csvCell(line,9)=="1" && csvCell(line,10)=="1","Valid triplet/color pair not recorded");}
    require(rows==drain.count(),"Evidence row count differs from consumed frame count");
}
void sourceAndInputFailures() {
    for(auto behavior:{Behavior::Throw,Behavior::BadSize,Behavior::NoImage}) {
        auto state=std::make_shared<State>();auto source=std::make_shared<FakeSource>(state,behavior);
        auto csv=std::make_shared<std::ostringstream>();
        r27::BackgroundDrain drain(source,csv);bool observed=false;
        until([&]{try{drain.check();}catch(const std::exception&){observed=true;}return observed;},"Runtime Source failure was not published");
        drain.stop();
        const char* wanted=behavior==Behavior::Throw?"source failure sentinel":
            behavior==Behavior::BadSize?"ORIGINAL_INPUT_CONTRACT":"no image";
        throwsContaining([&]{drain.check();},wanted);
        require(drain.count()==0,"Invalid source frame was counted as valid");
        require(state->active.load()==0,"Failure left Source running after join");
    }
}
void missingMetadataIsUnverified() {
    for(auto behavior:{Behavior::MissingMetadata,Behavior::MissingTimestamp}) {
        auto state=std::make_shared<State>();auto source=std::make_shared<FakeSource>(state,behavior);
        auto csv=std::make_shared<std::ostringstream>();r27::BackgroundDrain drain(source,csv);
        until([&]{return state->completed.load()>=3;},"Missing-metadata source did not run");
        drain.stop();drain.check();
        require(drain.count()>0 && drain.metadataVerifiedCount()==0,"Missing metadata falsely passed");
        require(drain.metadataUnverifiedCount()==drain.count(),"Missing metadata was not counted");
        std::istringstream input(csv->str());std::string line;std::getline(input,line);
        while(std::getline(input,line)) {
            require(line.find("background_metadata_unverified")!=std::string::npos,"Missing metadata issue was omitted");
            require(csvCell(line,9)=="0","Missing metadata evidence falsely passed");
        }
    }
}
void joinBeforeSourceDestruction() {
    auto state=std::make_shared<State>();std::weak_ptr<FakeSource> weak;
    {
        auto source=std::make_shared<FakeSource>(state,Behavior::Block);weak=source;
        auto csv=std::make_shared<std::ostringstream>();r27::BackgroundDrain drain(source,csv);source.reset();
        until([&]{return state->blocked.load();},"Source did not enter blocked call");
        std::atomic<bool> stopping{false},stopped{false};
        std::thread owner([&]{stopping.store(true);drain.stop();stopped.store(true);});
        until([&]{return stopping.load();},"Stop did not start");
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        const bool premature=stopped.load() || state->destroyed.load() || weak.expired();
        state->release.store(true);state->cv.notify_all();owner.join();
        require(!premature,"Source released while next() was in flight");
        require(stopped.load() && state->active.load()==0,"Stop did not join in-flight call");
        require(!weak.expired() && !state->destroyed.load(),"Source ownership ended before drain lifetime");
        drain.check();
    }
    require(state->destroyed.load() && weak.expired(),"Source ownership leaked after drain destruction");
}
void exceptionUnwindJoins() {
    auto state=std::make_shared<State>();auto source=std::make_shared<FakeSource>(state);
    try {
        auto csv=std::make_shared<std::ostringstream>();r27::BackgroundDrain drain(source,csv);
        until([&]{return state->completed.load()>=2;},"Unwind source did not start");
        throw std::runtime_error("owner exception sentinel");
    } catch(const std::runtime_error& e) {
        require(std::string(e.what())=="owner exception sentinel","Unexpected unwind exception");
    }
    const auto calls=state->calls.load();std::this_thread::sleep_for(std::chrono::milliseconds(4));
    require(state->active.load()==0 && state->calls.load()==calls,"RAII unwind did not stop/join acquisition");
    source.reset();require(state->destroyed.load(),"Source retained after RAII unwind");
}
void streamFailures() {
    { // Failure during writing must be visible to check() before stop().
        auto state=std::make_shared<State>();auto source=std::make_shared<FakeSource>(state);
        auto csv=std::make_shared<FaultStream>();r27::BackgroundDrain drain(source,csv);
        until([&]{return state->completed.load()>=2;},"Write-fault source did not start");
        csv->buffer.fail_writes.store(true);bool observed=false;
        until([&]{try{drain.check();}catch(const std::runtime_error& e){observed=std::string(e.what()).find("write failed")!=std::string::npos;}return observed;},"Runtime stream failure was not published");
        drain.stop();throwsContaining([&]{drain.check();},"write failed");
    }
    { // A buffered stream can fail only on final flush, after writer join.
        auto state=std::make_shared<State>();auto source=std::make_shared<FakeSource>(state);
        auto csv=std::make_shared<FaultStream>();csv->buffer.fail_sync.store(true);
        r27::BackgroundDrain drain(source,csv);
        until([&]{return state->completed.load()>=2;},"Flush-fault source did not start");
        drain.check();drain.stop();throwsContaining([&]{drain.check();},"flush failed");
        require(state->active.load()==0,"Flush failure skipped acquisition join");
        drain.stop(); // Idempotent and nonthrowing, including destruction.
    }
    { // A failed flush during stack unwinding must not replace the exception.
        auto state=std::make_shared<State>();auto source=std::make_shared<FakeSource>(state);
        auto csv=std::make_shared<FaultStream>();csv->buffer.fail_sync.store(true);
        throwsContaining([&]{r27::BackgroundDrain drain(source,csv);throw std::runtime_error("preserve owner exception");},"preserve owner exception");
        require(state->active.load()==0,"Destructor flush failure left worker running");
    }
}
}

int main() {
    try {
        normalConsumption();sourceAndInputFailures();missingMetadataIsUnverified();
        joinBeforeSourceDestruction();exceptionUnwindJoins();streamFailures();
        std::cout<<"PASS background drain: original input, stop/join, cross-thread errors, Source lifetime, metadata and stream failures\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
